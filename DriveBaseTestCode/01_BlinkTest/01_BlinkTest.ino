// The following code should blink a red light near the positive voltage wire
void setup() {
  pinMode(PC6, OUTPUT);
}
void loop() {
  digitalWrite(PC6, HIGH);
  delay(2000);
  digitalWrite(PC6, LOW);
  delay(2000);
}
