/*
* Copyright (c) 2015, Texas Instruments Incorporated
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* *  Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* *  Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* *  Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/** ============================================================================
*  @file       Board.h
*
*  @brief      CC2650EM_7ID Board Specific header file.
*              The project options should point to this file if this is the
*              CC2650EM you are developing code for.
*
*  The CC2650 header file should be included in an application as follows:
*  @code
*  #include <Board.h>
*  @endcode
*
*  ============================================================================
*/
#ifndef __CC2650EM_7ID_H__
#define __CC2650EM_7ID_H__

#ifdef __cplusplus
extern "C" {
#endif
  
  /** ============================================================================
  *  Symbol by generic Board.c to include the correct kit specific Board.c
  *  ==========================================================================*/
#define CC2650EM_7ID
  
  
  /** ============================================================================
  *  Includes
  *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>
  /** ============================================================================
  *  Externs
  *  ==========================================================================*/
extern PIN_Config BoardGpioInitTable[];
  
  /** ============================================================================
  *  Defines
  *  ==========================================================================*/
  
  /* Mapping of pins to board signals using general board aliases
  *      <board signal alias>                <pin mapping>
  */
  // Leds
#define Board_LED_ON                        1 /* LEDs on CC2650 are active high */
#define Board_LED_OFF                       0
#define Board_LED1                          IOID_6
#define Board_LED2                          IOID_7
#define Board_LED3                          PIN_UNASSIGNED
#define Board_LED4                          PIN_UNASSIGNED
  // Button Board
#define Board_KEY_SELECT                    PIN_UNASSIGNED
#define Board_KEY_UP                        IOID_1
#define Board_KEY_DOWN                      IOID_0
#define Board_KEY_LEFT                      PIN_UNASSIGNED
#define Board_KEY_RIGHT                     PIN_UNASSIGNED
  // LCD  Board
#define Board_3V3_EN                        PIN_UNASSIGNED
#define Board_LCD_MODE                      PIN_UNASSIGNED
#define Board_LCD_RST                       PIN_UNASSIGNED
#define Board_LCD_CSN                       PIN_UNASSIGNED
  // UART Board
#define Board_UART_RX                       IOID_2
#define Board_UART_TX                       IOID_3
#define Board_UART_CTS                      PIN_UNASSIGNED
#define Board_UART_RTS                      PIN_UNASSIGNED
  // SPI Board
#define Board_SPI0_MISO                     IOID_8
#define Board_SPI0_MOSI                     IOID_9
#define Board_SPI0_CLK                      IOID_10
#define Board_SPI0_CSN                      PIN_UNASSIGNED
#define Board_SPI1_MISO                     PIN_UNASSIGNED
#define Board_SPI1_MOSI                     PIN_UNASSIGNED
#define Board_SPI1_CLK                      PIN_UNASSIGNED
#define Board_SPI1_CSN                      PIN_UNASSIGNED
  // I2C 
#define Board_I2C0_SDA0                     IOID_25
#define Board_I2C0_SCL0             	    IOID_27
  //LORA
#ifdef TI_DRIVERS_LORA_ENABLE
#define Board_SPI_LORA_CS          			IOID_11
#define Board_LORA_RXEN						IOID_15
#define Board_LORA_TXEN						IOID_14
#define Board_LORA_RST						IOID_21
#define Board_LORA_INT1						IOID_22
#define Board_LORA_INT2						IOID_29
#define Board_LORA_INT3						IOID_12
#define Board_LORA_CS_ON           			0
#define Board_LORA_CS_OFF          			1
#define Board_LORA_RST_LOW           		0
#define Board_LORA_RST_HIGH          		1
#endif
#define TEST_PIN					        IOID_4

  /** ============================================================================
  *  Instance identifiers
  *  ==========================================================================*/
  /* Generic I2C instance identifiers */
#define Board_I2C                   CC2650_I2C0
  /* Generic SPI instance identifiers */
#define Board_SPI0                  CC2650_SPI0
#define Board_SPI1                  CC2650_SPI1
  /* Generic UART instance identifiers */
#define Board_UART                  CC2650_UART0
  /* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC2650_CRYPTO0
  /* Generic Watchdog instance identifiers */
#define Board_WATCHDOG              CC2650_WATCHDOG0
  
  
  /** ============================================================================
  *  Number of peripherals and their names
  *  ==========================================================================*/
  
  /*!
  *  @def    CC2650_I2CName
  *  @brief  Enum of I2C names on the CC2650 dev board
  */
  typedef enum CC2650_I2CName {
    CC2650_I2C0 = 0,
    CC2650_I2CCOUNT
  } CC2650_I2CName;
  
  /*!
  *  @def    CC2650_CryptoName
  *  @brief  Enum of Crypto names on the CC2650 dev board
  */
  typedef enum CC2650_CryptoName {
    CC2650_CRYPTO0 = 0,
    CC2650_CRYPTOCOUNT
  } CC2650_CryptoName;
  
  /*!
  *  @def    CC2650_SPIName
  *  @brief  Enum of SPI names on the CC2650 dev board
  */
  typedef enum CC2650_SPIName {
    CC2650_SPI0 = 0,
    //CC2650_SPI1,
    CC2650_SPICOUNT
  } CC2650_SPIName;
  
  /*!
  *  @def    CC2650_UARTName
  *  @brief  Enum of UARTs on the CC2650 dev board
  */
  typedef enum CC2650_UARTName {
    CC2650_UART0 = 0,
    CC2650_UARTCOUNT
  } CC2650_UARTName;
  
  /*!
  *  @def    CC2650_UdmaName
  *  @brief  Enum of DMA buffers
  */
  typedef enum CC2650_UdmaName {
    CC2650_UDMA0 = 0,
    CC2650_UDMACOUNT
  } CC2650_UdmaName;
  
typedef enum CC2650_GPTimerName {
    CC2650_GPTIMER0A = 0,
    CC2650_GPTIMER0B,
//  CC2650_GPTIMER1A,
//  CC2650_GPTIMER1B,
//  CC2650_GPTIMER2A,
//  CC2650_GPTIMER2B,
//  CC2650_GPTIMER3A,
//  CC2650_GPTIMER3B,
    CC2650_GPTIMERUNITSCOUNT
} CC2650_GPTimerName;

typedef enum CC2650_GPTimers {
    CC2650_GPTIMER0 = 0,
//  CC2650_GPTIMER1,
//  CC2650_GPTIMER2,
//  CC2650_GPTIMER3,
    CC2650_GPTIMERCOUNT
} CC2650_GPTimers;

typedef enum CC2650_PWM {
    CC2650_PWM0 = 0,    
    CC2650_PWM1,
//  CC2650_PWM2,
//  CC2650_PWM3,
//  CC2650_PWM4,
//  CC2650_PWM5,
//  CC2650_PWM6,
//  CC2650_PWM7,
    CC2650_PWMCOUNT
} CC2650_PWM;

#define Board_PWMPIN0  Board_PWM1
#define Board_PWMPIN1  PLC_SPK
#ifdef __cplusplus
}
#endif

#endif /* __CC2650EM_H__ */
