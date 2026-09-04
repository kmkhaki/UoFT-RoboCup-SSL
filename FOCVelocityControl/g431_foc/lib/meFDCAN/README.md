# meFDCAN

A CAN bus Arduino library for the stm32G473 FDCAN.  

## Description

meFDCAN is an Arduino library for the stm32G4 microcontroller family, especially the stm32G473, which allows communication with other CAN devices using the FDCAN module. The library provides a simple and intuitive interface for sending and receiving CAN messages, making it easy to integrate CAN communication into your stm32G473-based projects.  

The library is designed to work with the STM32 (official) Arduino Core, so you can easily use it with the Arduino Integrated Development Environment (IDE).

meFDCAN is designed to be simple, work with 'Classic' CAN but not the advanced features of fdCAN.  

## Features

- Simple and intuitive API for sending and receiving CAN messages.
- Supports Receive Interrupts.
- Supports message filtering.
- Classic CAN only.
- Supports 3 CAN ports (stm32G473...)

## Requirements

- An stm32G473 microcontroller board
- A CAN transceiver, such as the TJA1050 or the SN65HVD230.
- The Arduino IDE, version 1.8.x or higher.
- STM32 Core 2.4.0

## Design 

meFDCAN is for the most part an 'Old School' library using almost no Class objects.  The reason this direction was taken was largely to accomidate reliable interrupt servicing.  I found mixing the HAL with Class objects during Interrupt handling to be problematic.  I did want to use the HAL libraries, so it was decided not to use class objects.

## Usage

To use meFDCAN in your sketch, include the library header file:

```c++
#include <meFDCAN.h>
```

Two 'include' files are also required in your project folder.  Check out the examples. you can get copies from there.

- build_opt.h
- hal_conf_extra.h

### PA11, PA12

If you wish to use PA11 and PA12, From the menu ```<Tools><USB Support>``` should be set to ```None.```

### External Crystal
#### https://github.com/mackelec/STM32-External-Crystals---SystemClock_Config

Generic microcontroller selection with STM32 Arduino are mostly setup to use internal oscillators, as the stm32G473 is.  If you are going to use an external crystal, not a bad idea for CAN comms, then refer to my examples. The ```meClock_G473.h``` file can be found there.  This file is set up for a 8MHz crystal.  The ```build_opt.h``` defines the crystal is 8Mhz. Add the #include as shown in the main file.
```
#include "meClock_G473.h"
```

### meCANbuffer

A FIFO or circular buffer that can be used particularly for recording messages received from Interrupt Handlers.

https://github.com/mackelec/meCANbuffer

### initialisation


```
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
```
