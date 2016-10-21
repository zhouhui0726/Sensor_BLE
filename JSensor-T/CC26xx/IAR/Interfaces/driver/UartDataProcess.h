/******************************************************************************
* @file  UartDataProcess.h
*
*****************************************************************************/

#ifndef UART_DATA_PROCESS_H
#define UART_DATA_PROCESS_H
/*********************************************************************
* INCLUDES
*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
/*********************************************************************
*  EXTERNAL VARIABLES
*/
typedef struct
{
  uint16_t packethead;
  uint8_t len;
  uint8_t cmd;
  uint8_t payload[16];
	uint8_t crc;
} UartDataPacket;

typedef struct
{
  uint8_t cmd;
  uint8_t networkstat;
  uint8_t version[4];
	uint8_t number[8];
} UartReturnModuleInfo;
/*********************************************************************
* CONSTANTS
*/
#define UART_HEAD_DATA                  0x4A43


#define UART_CMD_WRITE_SERIAL_NUMBER    0x01
/*********************************************************************
* MACROS
*/
extern uint8_t Serial_number[9];
/*********************************************************************
* FUNCTIONS
*/
extern void TLwrite (uint8_t *buf, uint8_t len);
extern void UartDataProcess(uint8_t *data, uint8_t len);
extern bool Uart_GeneralModuleDataProcess(uint8_t* data, uint8_t len);
extern uint8_t *Get_ModuleVersion(void);
#endif
#endif //UARTQUEUE_H