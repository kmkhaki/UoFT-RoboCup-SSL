// esp32_can_test/src/main.cpp
// VELOCITY COMMAND (v4, BLE-driven): receives the joystick-derived velocity
// (-128..127) from the Feather 32u4 over BLE - this board is the BLE CENTRAL,
// connecting to the Feather's peripheral UART service (see
// feather32u4_joystick_ble and esp32_ble_central_test, which this merges
// together) - and forwards it to ESC1 over CAN, auto-repeated every
// SEND_INTERVAL_MS so ESC1's 500ms fail-safe timeout stays satisfied even
// between BLE updates.
//
// Two independent fail-safes protect against a stale/disconnected upstream:
//   1. If the BLE link is lost (or a value hasn't been heard in
//      BLE_TIMEOUT_MS), this board zeroes the target velocity itself.
//   2. ESC1 has its own 500ms CAN-side fail-safe as a second, independent
//      layer, in case this board itself hangs/crashes.
//
// Built on the ESP-IDF native TWAI driver (CAN) + the ESP32 BLE Arduino
// library (Bluetooth) - no local joystick wiring on this board anymore, that
// moved to the Feather.

#include <Arduino.h>
#include <BLEDevice.h>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_35

#define CAN_ID_VELOCITY 0x120

#define VELOCITY_MIN -128
#define VELOCITY_MAX 127

// Repeat well under ESC1's 500ms fail-safe timeout, so a couple of
// consecutive dropped frames won't trip a stop on their own.
#define SEND_INTERVAL_MS 150

// If no BLE velocity update has arrived in this long, zero the target
// ourselves rather than keep repeating a stale value - independent of, and
// in addition to, ESC1's own 500ms CAN-side fail-safe.
#define BLE_TIMEOUT_MS 500

static BLEUUID ServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID CharUUID_RX("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");  // write TO the Feather (reserved, unused)
static BLEUUID CharUUID_TX("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");  // notify FROM the Feather - what we want

static BLEAddress *pServerAddress = nullptr;
static bool DoConnect = false;
static bool Connected = false;
static BLERemoteCharacteristic *pTXCharacteristic = nullptr;
static BLERemoteCharacteristic *pRXCharacteristic = nullptr;
static String RxLineBuffer;

// Shared between the BLE callback context (a different FreeRTOS task than
// loop()) and loop() itself - volatile for cross-task visibility, same
// pattern used for the ISR-shared variables on the ESC1 side.
volatile int8_t CurrentVelocity = 0;
volatile bool Button1 = false, Button2 = false, Button3 = false;
volatile uint32_t LastBleMsgMillis = 0;

uint32_t LastSendMillis = 0;

// --- loop() responsiveness instrumentation (temporary, for the BLE/CAN
// jerkiness diagnostic) -----------------------------------------------------
// The prebuilt Arduino-ESP32 framework doesn't have FreeRTOS's per-task
// runtime-stats option enabled, so a true "% CPU used by the Bluetooth
// stack" number isn't available without rebuilding the framework. What we
// CAN measure directly is how promptly loop() itself gets to run - the gap
// between successive loop() entries - which is what actually determines
// whether the 150ms CAN heartbeat goes out on time. A large gap here means
// something (BLE processing, a blocking call, etc.) is starving loop().
static uint32_t LoopCount = 0;
static uint32_t LastLoopMicros = 0;
static uint32_t MaxGapMicros = 0;
static uint64_t GapSumMicros = 0;
static uint32_t LastStatsReportMillis = 0;

static void RecordLoopTiming() {
  uint32_t NowMicros = micros();
  if (LastLoopMicros != 0) {
    uint32_t Gap = NowMicros - LastLoopMicros;
    GapSumMicros += Gap;
    if (Gap > MaxGapMicros) MaxGapMicros = Gap;
  }
  LastLoopMicros = NowMicros;
  LoopCount++;

  uint32_t ElapsedMs = millis() - LastStatsReportMillis;
  if (ElapsedMs >= 2000) {
    float Hz = (LoopCount * 1000.0f) / ElapsedMs;
    uint32_t AvgGapMicros = (LoopCount > 1) ? (uint32_t)(GapSumMicros / (LoopCount - 1)) : 0;
    Serial.printf("[PERF] loop=%.0fHz  avgGap=%luus  maxGap=%luus  core=%d  freeHeap=%lu\n",
                  Hz, (unsigned long)AvgGapMicros, (unsigned long)MaxGapMicros, xPortGetCoreID(),
                  (unsigned long)ESP.getFreeHeap());
    LoopCount = 0;
    GapSumMicros = 0;
    MaxGapMicros = 0;
    LastStatsReportMillis = millis();
  }
}

static void SendVelocity(int8_t Velocity, bool Verbose) {
  twai_message_t TxMsg = {};
  TxMsg.identifier = CAN_ID_VELOCITY;
  TxMsg.data_length_code = 1;
  TxMsg.data[0] = (uint8_t)Velocity;

  esp_err_t Result = twai_transmit(&TxMsg, pdMS_TO_TICKS(100));
  LastSendMillis = millis();

  if (Verbose || Result != ESP_OK) {
    Serial.printf("Sent velocity=%d -> twai_transmit result: %s\n", Velocity,
                  Result == ESP_OK ? "OK" : esp_err_to_name(Result));
  }
}

// Feather sends "velocity,btn1,btn2,btn3" (e.g. "-45,0,1,0") - velocity is
// parsed the same way regardless of whether the button fields are present,
// since String::toInt() just stops at the first non-numeric character (the
// comma). The button fields are optional here on purpose: if the Feather's
// message format ever changes, velocity forwarding keeps working either way.
static void NotifyCallback(BLERemoteCharacteristic *Chr, uint8_t *pData, size_t Length, bool IsNotify) {
  for (size_t i = 0; i < Length; i++) {
    char c = (char)pData[i];
    if (c == '\n' || c == '\r') {
      if (RxLineBuffer.length() > 0) {
        int Comma1 = RxLineBuffer.indexOf(',');
        String VelocityField = (Comma1 >= 0) ? RxLineBuffer.substring(0, Comma1) : RxLineBuffer;

        long Value = VelocityField.toInt();
        if (Value < VELOCITY_MIN) Value = VELOCITY_MIN;
        if (Value > VELOCITY_MAX) Value = VELOCITY_MAX;
        CurrentVelocity = (int8_t)Value;
        LastBleMsgMillis = millis();

        if (Comma1 >= 0) {
          String Rest = RxLineBuffer.substring(Comma1 + 1);
          int Comma2 = Rest.indexOf(',');
          int Comma3 = (Comma2 >= 0) ? Rest.indexOf(',', Comma2 + 1) : -1;
          if (Comma2 >= 0 && Comma3 >= 0) {
            Button1 = (Rest.substring(0, Comma2).toInt() != 0);
            Button2 = (Rest.substring(Comma2 + 1, Comma3).toInt() != 0);
            Button3 = (Rest.substring(Comma3 + 1).toInt() != 0);
          }
        }

        Serial.printf("Received velocity: %d  buttons: %d %d %d\n", CurrentVelocity, Button1, Button2, Button3);
        SendVelocity(CurrentVelocity, false);  // forward immediately, don't wait for the next heartbeat tick
        RxLineBuffer = "";
      }
    } else {
      RxLineBuffer += c;
    }
  }
}

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *pClient) override {}
  void onDisconnect(BLEClient *pClient) override {
    Connected = false;
    Serial.println("BLE disconnected from Feather - will rescan.");
  }
};

static bool ConnectToServer(BLEAddress Address) {
  Serial.print("Connecting to: ");
  Serial.println(Address.toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new ClientCallbacks());

  if (!pClient->connect(Address)) {
    Serial.println("Connection attempt failed");
    return false;
  }
  Serial.println("Connected to Feather");

  BLERemoteService *pRemoteService = pClient->getService(ServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find UART service on this device");
    pClient->disconnect();
    return false;
  }

  pTXCharacteristic = pRemoteService->getCharacteristic(CharUUID_TX);
  if (pTXCharacteristic == nullptr) {
    Serial.println("Failed to find TX characteristic");
    pClient->disconnect();
    return false;
  }

  if (pTXCharacteristic->canNotify()) {
    pTXCharacteristic->registerForNotify(NotifyCallback);
    // Explicit CCCD write - registerForNotify() alone doesn't reliably
    // enable notifications on every version of this library.
    BLERemoteDescriptor *pCCCD = pTXCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (pCCCD != nullptr) {
      uint8_t NotifyOn[] = {0x1, 0x0};
      pCCCD->writeValue(NotifyOn, 2, true);
    }
  } else {
    Serial.println("TX characteristic doesn't support notify - unexpected");
  }

  pRXCharacteristic = pRemoteService->getCharacteristic(CharUUID_RX);  // reserved, unused for now

  LastBleMsgMillis = millis();  // don't let the fail-safe trip immediately on connect, before the first real value
  return true;
}

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice AdvertisedDevice) override {
    if (AdvertisedDevice.haveServiceUUID() && AdvertisedDevice.getServiceUUID().equals(ServiceUUID)) {
      Serial.print("Found the Feather: ");
      Serial.println(AdvertisedDevice.toString().c_str());
      AdvertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(AdvertisedDevice.getAddress());
      DoConnect = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 VELOCITY COMMAND (v4): BLE-driven (Feather joystick) -> CAN, auto-repeating");

  twai_general_config_t GConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t TConfig = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t FConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&GConfig, &TConfig, &FConfig) != ESP_OK) {
    Serial.println("FATAL: twai_driver_install failed");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("FATAL: twai_start failed");
    while (1) delay(1000);
  }
  Serial.println("CAN ready.");

  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);

  Serial.println("Scanning for the Feather joystick...");
}

void loop() {
  RecordLoopTiming();

  // Drain unexpected CAN RX (not expecting anything back in this direction,
  // but keep the queue clear and surface anything unexpected for visibility).
  twai_message_t RxMsg;
  while (twai_receive(&RxMsg, 0) == ESP_OK) {
    Serial.printf("Received unexpected CAN ID 0x%lx\n", (unsigned long)RxMsg.identifier);
  }

  if (DoConnect) {
    if (ConnectToServer(*pServerAddress)) {
      Connected = true;
      Serial.println("Ready - forwarding joystick velocity to CAN.");
    } else {
      Serial.println("Connect failed - rescanning...");
    }
    DoConnect = false;
  }

  // BLEScan::start() blocks the caller until either a match calls stop()
  // from inside onResult() or the duration elapses, so by the time we get
  // back here any previous scan has already ended one way or the other.
  if (!Connected && !DoConnect) {
    Serial.println("Scanning...");
    BLEDevice::getScan()->start(30, false);
  }

  // BLE fail-safe: zero the target if we haven't heard from the Feather
  // recently - covers both "never connected yet" and "connected but the
  // link went quiet" - independent of ESC1's own CAN-side fail-safe.
  if (millis() - LastBleMsgMillis > BLE_TIMEOUT_MS && CurrentVelocity != 0) {
    CurrentVelocity = 0;
    Serial.println("BLE timeout - zeroing target velocity.");
  }

  if (millis() - LastSendMillis >= SEND_INTERVAL_MS) {
    SendVelocity(CurrentVelocity, false);  // silent heartbeat repeat
  }
}
