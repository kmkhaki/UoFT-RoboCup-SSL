#pragma once



#include <arduino.h>
#define PROC_G4 340

#ifdef STM32G4xx
  #define _SERIES PROC_G4
#endif

/**
 * @brief   Initialize the FDCAN module
 *
 * @param   bitrate  The desired bitrate for the FDCAN module (default = 125)
 * @param   canPort  The port number for the FDCAN module (default = 1)
 * @param   rxPin    The pin number for the receive pin (default = PA11)
 * @param   txPin    The pin number for the transmit pin (default = PA12)
 *
 * @return  true if the initialization was successful, false otherwise
 */

bool meFDCAN_init(int bitrate=125,int canPort=1,int rxPin=PA11,int txPin=PA12);

/**
 * @brief   Transmit data over the FDCAN module
 *
 * @param   txId     The ID for the transmission
 * @param   txData   The data to be transmitted
 * @param   canPort  The port number for the FDCAN module (default = 1)
 * @param   len      The length of the data to be transmitted (default = 8)
 *
 * @return  true if the transmission was successful, false otherwise
 */

bool meFDCAN_transmit(int txId, uint8_t *txData,int canPort=1, int len = 8);

/**
 * @brief   Receive data over the FDCAN module
 *
 * @param   id        The ID of the received data
 * @param   len       The length of the received data
 * @param   rxData    The received data
 * @param   canPort   The port number for the FDCAN module (default = 1)
 * @param   type      The type of the received message (default = nullptr)
 * @param   fifo      The FIFO number to use for receiving (default = 0)
 *
 * @return  The index of the received data
 */

int  meFDCAN_receive(uint32_t *id,uint8_t *len,uint8_t rxData[],int canPort=1,int *type=nullptr,int fifo=0);

/**
 * @brief   Set the receive interrupt for the FDCAN module
 * 
 * @param   canPort  The port number for the FDCAN module (default = 1)
 * @param   fifo     The FIFO number to use for the interrupt (default = 0)
 * 
 * @return  true if the setting of the receive interrupt was successful, false otherwise
 */

bool me_FDCAN_setRxInterrupt(int canPort=1,int fifo=0);

/**
    @brief Add a filter for the FDCAN module
    @param id1 The first ID for the filter
    @param id2 The second ID for the filter
    @param filtertype The type of the filter  (default = FDCAN_FILTER_RANGE)
    (options: FDCAN_FILTER_RANGE, FDCAN_FILTER_DUAL, FDCAN_FILTER_MASK)
    @param canPort The port number for the FDCAN module (default = 1)
    @return The index of the added filter
*/


int  me_FDCAN_addFilter(uint32_t id1,uint32_t id2,uint32_t filtertype=FDCAN_FILTER_RANGE,int canPort=1); 


//------------   END of User Functions ----------

/*
 *      Bir Rate Parameters - CAN Controller
 * 

//-- http://www.bittiming.can-wiki.info/


/**
 * @brief   Struct to hold the bit rate settings for the CAN bus
 * 
 * Clock:     The clock frequency for the CAN bus (MHz)
 * bitrate:   The desired bit rate for the CAN bus (kHz)
 * prescale:  The prescaler value for the CAN bus
 * Seg1:      The segment 1 value for the CAN bus
 * Seg2:      The segment 2 value for the CAN bus
 */

struct bitRateSetting
{
  int Clock;
  int bitrate;
  int prescale;
  int Seg1;
  int Seg2;
};

/**
 * @brief   Global variable to hold the current bit rate setting for the CAN bus
 */
static bitRateSetting  canSetting;

/**
 * @brief   Array of bit rate settings for the CAN bus
 */
bitRateSetting brSettings[] = {
                              {150,50,12,218,31},
                              {150,125,40,25,4},
                              {150,250,3,174,25},
                              {150,500,2,130,19},
                              {150,1000,1,130,19},
                              // CUSTOM: added for the B-G431B-ESC1's actual FDCAN kernel clock
                              // (170MHz, confirmed via direct register readback earlier this
                              // session - NOT the 150MHz this table otherwise assumes). Values
                              // match SimpleCanLib's own verified-correct 250kbit/s config on
                              // this exact board: prescaler=40, TSEG1=14, TSEG2=2 -> 17 tq/bit,
                              // tq=235.3ns, bit time=4us -> exactly 250,000 bit/s.
                              {170,250,40,14,2}
                              };




static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_HandleTypeDef hfdcan2;
static FDCAN_HandleTypeDef hfdcan3;


struct meFDCAN_stdFilter
{
  bool filter;
  uint32_t filterindex;
  uint32_t filtertype;
  uint32_t id1;
  uint32_t id2;
};



class Filter
{
public:
  meFDCAN_stdFilter stdfilters[18];
  bool filtered;

  int addFilter(meFDCAN_stdFilter filter) 
  {
    int i = returnNextFilterIndex();
    if (i >= 0 && i < 18) 
    {
      filtered = true;
      stdfilters[i].filter = true;
      stdfilters[i].filterindex = NewFilterIndex;
      stdfilters[i].filtertype = filter.filtertype;
      stdfilters[i].id1 = filter.id1;
      stdfilters[i].id2 = filter.id2;
      NewFilterIndex ++;
      return index;
    } 
    else 
    {
      return -1;
    }
  }
  int returnNextFilterIndex()
  {
    for (int i = 0; i < 18; i++)
    {
      if (!stdfilters[i].filter)
      {
        return i;
      }
    }
    return -1;
  }
  Filter()
  {
    index=0;
    NewFilterIndex = 0;
    filtered = false;
    for (int i = 0; i < 18; i++)
    {
      stdfilters[i].filter = false;
    }
  }
  
  void startFilter()
  {
    index = 0;
  }

  meFDCAN_stdFilter nextFilter()
  {
    if (index >= 0 && index < 18 && stdfilters[index].filter)
    {
      meFDCAN_stdFilter currentFilter = stdfilters[index];
      index++;
      return currentFilter;
    }
    else
    {
      return meFDCAN_stdFilter();
    }
  }

private:
int index;                //-- index of next filter to send
uint32_t NewFilterIndex;  //-- used when Adding new Filter
  
}; //---- END class Filter

class Filters
{
public:
  Filter filter[3];
  
  bool isFilter(int canPort)
  {
    if (canPort >= 1 && canPort <= 3)
      return filter[canPort - 1].filtered;
  }
  int addFilter(int canPort,meFDCAN_stdFilter newfilter)
  {
    if (canPort >= 1 && canPort <= 3)
      return filter[canPort - 1].addFilter(newfilter);
    else
      return -1;
  }
  void startFilter(int canPort)
  {
    if (canPort >= 1 && canPort <= 3)
      filter[canPort - 1].startFilter();
    //else
      // error handling code here
  }

  meFDCAN_stdFilter nextFilter(int canPort)
  {
    if (canPort >= 1 && canPort <= 3)
      return filter[canPort - 1].nextFilter();
    //else
      // error handling code here
  }
};  //----- END of class Filters


Filters canFilters;




static bool setupFDCANpin(int);
static int findCanSettings(int, int);
static FDCAN_HandleTypeDef*  canPortHandle(int );
static uint32_t int_to_fdcan_dlc(int ); 
static uint8_t rxDataLength(uint32_t );
static bool meFDCAN_setupFilters(int);
static bool meFDCAN_addConfigFilter(int canPort,meFDCAN_stdFilter * mefilter);
static uint8_t rxDataLengthFD(uint32_t _data_length, bool FD=false); 

/*******************************************************
 * 
 *            CAN INIT
 * 
 *******************************************************/

bool meFDCAN_init(int bitrate,int canPort,int rxPin,int txPin)
{
  FDCAN_HandleTypeDef *fdcan;
  switch (canPort)
  {
    case 1: hfdcan1.Instance = FDCAN1;fdcan = &hfdcan1;break;
    // CUSTOM: FDCAN2/FDCAN3 don't exist on the G431 (single-FDCAN chip) - only
    // canPort==1 is usable here.
    default: return (false);break;
  }
  bool success = true;
  
  RCC_PeriphCLKInitTypeDef periphClkInit = { };
  HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);
  
  // Initializes the peripherals clocks
  periphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
  periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&periphClkInit) != HAL_OK)
  {
    //Error_Handler();
    Serial.println("HAL_RCC FAIL");
    return (false);
  }

  // Peripheral clock enable
  __HAL_RCC_FDCAN_CLK_ENABLE();
  //__HAL_RCC_GPIOB_CLK_ENABLE();

  if (!setupFDCANpin(rxPin)) return (false);
  if (!setupFDCANpin(txPin)) return (false);

  FDCAN_InitTypeDef *init = &fdcan->Init;
  
  init->ClockDivider = FDCAN_CLOCK_DIV1;
  init->FrameFormat = FDCAN_FRAME_CLASSIC; //FDCAN_FRAME_FD_BRS; FDCAN_FRAME_FD_NO_BRS
  init->Mode = FDCAN_MODE_NORMAL;
  init->AutoRetransmission = DISABLE;
  init->TransmitPause = ENABLE;
  init->ProtocolException = DISABLE;

  int canClock;
  uint32_t pclk1_frequency = HAL_RCC_GetPCLK1Freq();
  canClock = (int)pclk1_frequency/1000000;
  int index = findCanSettings(canClock,bitrate);
  if (index < 0) return false;

  
  /* Configure the CAN peripheral */
  
  init->NominalSyncJumpWidth = 1;
  
  init->NominalPrescaler = canSetting.prescale;
  init->NominalTimeSeg1 = canSetting.Seg1;
  init->NominalTimeSeg2 = canSetting.Seg2;
  //--- not used in Normal 
  init->DataPrescaler = canSetting.prescale;;
  init->DataSyncJumpWidth = 1;
  init->DataTimeSeg1 = canSetting.Seg1;
  init->DataTimeSeg2 = canSetting.Seg2;
  //-----
  init->StdFiltersNbr = 18;
  init->ExtFiltersNbr = 0;
  init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(fdcan) != HAL_OK)
  {
    return (false);
  }
  //----------  Filters Setup
  meFDCAN_setupFilters(canPort);
  
  if (HAL_FDCAN_Start(fdcan) != HAL_OK)
  {
    return (false);
  }
  return (true);
}                  //---------       END of Init


/*****************
 * 
 *   RECEIVE
 * 
 *****************/
 
int  meFDCAN_receive(uint32_t *id,uint8_t *len,uint8_t rxData[], int canPort,int *type,int fifo)
{
  FDCAN_HandleTypeDef *fdcan = canPortHandle(canPort);
  if (fdcan == NULL ) return (-1);
  uint32_t _fifo = FDCAN_RX_FIFO0;
  if (fifo == 1) _fifo = FDCAN_RX_FIFO1;
  int lvl = (int) HAL_FDCAN_GetRxFifoFillLevel(fdcan, _fifo);
  if (lvl == 0) return (lvl);
  
  FDCAN_RxHeaderTypeDef rxHeader;
  if (HAL_FDCAN_GetRxMessage(fdcan, _fifo, &rxHeader, rxData) != HAL_OK) return (-1);
  *id = rxHeader.Identifier;
  if (type != nullptr)
    {
        if (rxHeader.IdType == FDCAN_STANDARD_ID) *type = 1;
        if (rxHeader.IdType == FDCAN_EXTENDED_ID) *type = -1;
    }
  //if (rxHeader.IdType != FDCAN_STANDARD_ID) {return -2;}
  //*len = rxDataLength(rxHeader.DataLength);
  
  *len = rxDataLengthFD(rxHeader.DataLength,false);
  return (lvl);
}

/****************************
 * 
 *       Transmit
 * 
 ****************************/

bool meFDCAN_transmit(int txId, uint8_t *txData,int canPort, int len)
{
  FDCAN_HandleTypeDef *fdcan = canPortHandle(canPort);
  if (fdcan == NULL ) return (false);
  bool success = true;
  uint32_t dlc = int_to_fdcan_dlc(len);
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = txId;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = dlc;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader,  txData) != HAL_OK)
  {
    success = false;
  }
  return (success);
}
/*************************************************************************************
 *                                  Filter Functions
 *************************************************************************************/

int me_FDCAN_addFilter(uint32_t id1,uint32_t id2,uint32_t filtertype,int canPort)
{
  meFDCAN_stdFilter filter;
  filter.filterindex = 0;
  filter.id1 = id1;
  filter.id2 = id2;
  filter.filtertype = filtertype;
  return canFilters.addFilter(canPort,filter);
}



bool meFDCAN_setupFilters(int canPort)
{
  if (canPort <1 || canPort >3) return false;
  if (! canFilters.isFilter(canPort)) return (true);
  while (1)
  {
    meFDCAN_stdFilter filter = canFilters.nextFilter(canPort);
    if (filter.filter == false)    break;
    bool ans = meFDCAN_addConfigFilter(canPort,&filter);
    if (!ans) 
    {
      return (false);
    }
  }
  HAL_StatusTypeDef result;
  FDCAN_HandleTypeDef *fdcan = canPortHandle(canPort);
  result = HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  if (result != HAL_OK) 
  {
    return (false);
  }
  return (true);
}




bool meFDCAN_addConfigFilter(int canPort,meFDCAN_stdFilter * mefilter)
{
  FDCAN_HandleTypeDef *fdcan = canPortHandle(canPort);
  FDCAN_FilterTypeDef filter_config;
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = mefilter->filterindex;
  filter_config.FilterType = mefilter->filtertype;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = mefilter->id1;
  filter_config.FilterID2 = mefilter->id2;
  if(HAL_FDCAN_ConfigFilter(fdcan, &filter_config) != HAL_OK)
  {
      // handle error
      Serial.println("BAD FILTER 2");
      return (false);
  }  
  return (true);
}



/**********************************************************
 *                 Interrupts
 **********************************************************/

bool me_FDCAN_setRxInterrupt(int canPort,int fifo)
{
  IRQn_Type irq;
  FDCAN_HandleTypeDef *fdcan = canPortHandle(canPort);
  if (fdcan == NULL ) return (false);
  bool success = true;
  uint32_t _fifo = FDCAN_RX_FIFO0;
  if (fifo == 1) _fifo = FDCAN_RX_FIFO1;
  if (HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
      return (false);
  }
  if (canPort == 1) {irq = FDCAN1_IT0_IRQn;}
  // CUSTOM: FDCAN2/FDCAN3 IRQs don't exist on the G431 (single-FDCAN chip) - we
  // only ever use canPort==1 anyway.
  
  /* Enable FDCAN IRQ */
  HAL_NVIC_SetPriority(irq, 0, 0);
  HAL_NVIC_EnableIRQ(irq);
  return (true);
}


/****************************************************
 *                  Misc
 ****************************************************/
 
FDCAN_HandleTypeDef* canPortHandle(int canPort)
{
  FDCAN_HandleTypeDef *fdcan=NULL;
  
  switch (canPort)
  {
    case 1: fdcan = &hfdcan1;break;
    case 2: fdcan = &hfdcan2;break;
    case 3: fdcan = &hfdcan3;break;
  }
  return fdcan;
}

int meFDCAN_Port(FDCAN_HandleTypeDef * fdcan)
{
  if (fdcan->Instance == FDCAN1)
  {
    return (1);
  }
  // CUSTOM: FDCAN2/FDCAN3 don't exist on the G431 (single-FDCAN chip).
  return (-1);
}

bool setupFDCANpin(int _pin)
{
  bool success = true;
  GPIO_InitTypeDef gpio_init;
  switch (_SERIES)
  {
    case PROC_G4:
      switch (_pin)
      {
        //  CAN1
        case PA11: // rx
        __HAL_RCC_GPIOA_CLK_ENABLE();
          gpio_init.Pin = GPIO_PIN_11;
          gpio_init.Mode = GPIO_MODE_AF_PP;
          gpio_init.Pull = GPIO_NOPULL;
          gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
          gpio_init.Alternate = GPIO_AF9_FDCAN1;
          HAL_GPIO_Init(GPIOA, &gpio_init);
        break;
        case PA12: // tx
        __HAL_RCC_GPIOA_CLK_ENABLE();
          gpio_init.Pin = GPIO_PIN_12;
          gpio_init.Mode = GPIO_MODE_AF_PP;
          gpio_init.Pull = GPIO_NOPULL;
          gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
          gpio_init.Alternate = GPIO_AF9_FDCAN1;
          HAL_GPIO_Init(GPIOA, &gpio_init);
        break;
        
        
        case PB8: // rx
          __HAL_RCC_GPIOB_CLK_ENABLE();
          gpio_init.Pin = GPIO_PIN_8;
          gpio_init.Mode = GPIO_MODE_AF_PP;
          gpio_init.Pull = GPIO_NOPULL;
          gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
          gpio_init.Alternate = GPIO_AF9_FDCAN1;
          HAL_GPIO_Init(GPIOB, &gpio_init);
        break;
        case PB9: // tx
          __HAL_RCC_GPIOB_CLK_ENABLE();
          gpio_init.Pin = GPIO_PIN_9;
          gpio_init.Mode = GPIO_MODE_AF_PP;
          gpio_init.Pull = GPIO_NOPULL;
          gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
          gpio_init.Alternate = GPIO_AF9_FDCAN1;
          HAL_GPIO_Init(GPIOB, &gpio_init);
        break;

        case 49:  // PD0 // rx
          __HAL_RCC_GPIOD_CLK_ENABLE();
          gpio_init.Pin = GPIO_PIN_0;
          gpio_init.Mode = GPIO_MODE_AF_PP;
          gpio_init.Pull = GPIO_NOPULL;
          gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
          gpio_init.Alternate = GPIO_AF9_FDCAN1;
          HAL_GPIO_Init(GPIOD, &gpio_init);
        break;

        case 50:  // PD1 // tx
          __HAL_RCC_GPIOD_CLK_ENABLE();
          gpio_init.Pin = GPIO_PIN_1;
          gpio_init.Mode = GPIO_MODE_AF_PP;
          gpio_init.Pull = GPIO_NOPULL;
          gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
          gpio_init.Alternate = GPIO_AF9_FDCAN1;
          HAL_GPIO_Init(GPIOD, &gpio_init);
        break;

        // CUSTOM: removed the CAN2 (PB5/PB6/PB12/PB13) and CAN3 (PA8/PA15/PB3/PB4)
        // pin cases here - the G431 (this board's chip) only has one FDCAN
        // peripheral (FDCAN1), so FDCAN2/FDCAN3 and their GPIO_AF9_FDCAN2/
        // GPIO_AF11_FDCAN3 alternate-function macros don't exist in its CMSIS
        // headers and fail to compile. We only ever use PA11/PB9 (CAN1) anyway.
        break;

        default:
          success = false;
          break;
      }
    break;
    
  }
  return (success);
}

int findCanSettings(int _clock, int _bitRate)
{
   for (int i=0;i<(sizeof(brSettings)/sizeof(bitRateSetting));i++)
   {
      if (brSettings[i].Clock != _clock || brSettings[i].bitrate != _bitRate)
      {
        continue;
      }
      canSetting.prescale = brSettings[i].prescale;
      canSetting.Seg1 = brSettings[i].Seg1;
      canSetting.Seg2 = brSettings[i].Seg2;
      return i;
   }
   return -1;
}



uint8_t rxDataLengthFD(uint32_t _data_length, bool FD) 
{
  union DataLength 
  {
    uint32_t value;
    struct 
    {
      uint16_t ignored_bits;
      uint8_t data_length:4;
      uint8_t ignored:4;
    } classic;
    struct 
    {
      uint16_t ignored_bits;
      uint8_t data_length:3;
      uint8_t FD:1;
      uint8_t ignored:4;
    } FD;
  };

  DataLength dataLength;
  dataLength.value = _data_length;

  uint8_t len = dataLength.classic.data_length;
  if (len > 8) len = 8;

  if (FD && dataLength.classic.data_length > 8) 
  {
    switch (dataLength.FD.data_length) 
    {
      case 1:        len = 12;        break;
      case 2:        len = 16;        break;
      case 3:        len = 20;        break;
      case 4:        len = 24;        break;
      case 5:        len = 32;        break;
      case 6:        len = 48;        break;
      case 7:        len = 64;        break;
      default:
        break;
    }
  }

  return len;
}


uint32_t int_to_fdcan_dlc(int dlc) 
{
  switch (dlc) {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    case 8: return FDCAN_DLC_BYTES_8;
    case 12: return FDCAN_DLC_BYTES_12;
    case 16: return FDCAN_DLC_BYTES_16;
    case 20: return FDCAN_DLC_BYTES_20;
    case 24: return FDCAN_DLC_BYTES_24;
    case 32: return FDCAN_DLC_BYTES_32;
    case 48: return FDCAN_DLC_BYTES_48;
    case 64: return FDCAN_DLC_BYTES_64;
    default: return FDCAN_DLC_BYTES_0;
  }
}





 
