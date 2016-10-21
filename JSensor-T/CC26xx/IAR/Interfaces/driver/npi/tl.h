/**
  @headerfile:    peripheral.h
  $Date: 2014-05-22 14:49:51 -0700 (Thu, 22 May 2014) $
  $Revision: 38618 $

  @mainpage Transport Layer implementation

  Copyright 2009 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*/

#ifndef TL_H
#define TL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * Profile Callbacks
 */

// Callback when data needs to be parsed from TL
typedef void (*TLpacketparser_t)( void );

typedef struct
{
  TLpacketparser_t        pfnTLpacketparser;  // Called there is data to parse
} TLCBs_t;

/*********************************************************************
 * Function APIs
 */

extern void TL_handleISRevent(void);

extern void TLinit(ICall_Semaphore *pAppSem, TLCBs_t *appCallbacks,
                  uint16_t TX_DONE_EVENT, uint16_t RX_EVENT, uint16_t MRDY_EVENT );

extern void TLwrite(uint8_t *buf, uint8_t len);
extern void TLread(uint8_t *buf, uint8_t len);
extern uint16_t TLgetRxBufLen (void);

extern bool PTM_Uart_enable;

void NPITLUART_TransportTimeoutProcess(void);
#ifdef __cplusplus
}
#endif

#endif /* TL_H */