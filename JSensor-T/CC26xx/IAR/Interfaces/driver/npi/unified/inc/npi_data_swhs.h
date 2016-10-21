/*
 * npi_data_swhs.h
 *
 * NPI Data structures
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef NPI_DATA_H
#define NPI_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************

#include <stdint.h>
  
#include <ti/drivers/UART.h>
#include <ti/drivers/SPI.h>

// ****************************************************************************
// defines
// ****************************************************************************  

//! \brief NPI Constants
//! 
#define NPI_MSG_SOF_VAL                         0xFE
  
//! \brief NPI Command Type.
#define NPI_MSG_TYPE_POLL                       0x00
#define NPI_MSG_TYPE_SYNCREQ                    0x01
#define NPI_MSG_TYPE_ASYNC                      0x02
#define NPI_MSG_TYPE_SYNCRSP                    0x03

//! \brief NPI Message Lengths
#define NPI_MSG_CMD_LENGTH                      2
#define NPI_MSG_LEN_LENGTH                      2
#define NPI_MSG_HDR_LENGTH                      NPI_MSG_CMD_LENGTH + \
                                                NPI_MSG_LEN_LENGTH
  
//! \brief NPI Subsystem IDs
//!
#define RPC_SYS_RES0                            0
#define RPC_SYS_SYS                             1
#define RPC_SYS_MAC                             2
#define RPC_SYS_NWK                             3
#define RPC_SYS_AF                              4
#define RPC_SYS_ZDO                             5
#define RPC_SYS_SAPI                            6
#define RPC_SYS_UTIL                            7
#define RPC_SYS_DBG                             8
#define RPC_SYS_APP                             9
#define RPC_SYS_RCAF                            10
#define RPC_SYS_RCN                             11
#define RPC_SYS_RCN_CLIENT                      12
#define RPC_SYS_BOOT                            13
#define RPC_SYS_ZIPTEST                         14
#define RPC_SYS_DEBUG                           15
#define RPC_SYS_PERIPHERALS                     16
#define RPC_SYS_NFC                             17
#define RPC_SYS_PB_NWK_MGR                      18
#define RPC_SYS_PB_GW                           19
#define RPC_SYS_PB_OTA_MGR                      20
#define RPC_SYS_BLE_SNP                         21
#define RPC_SYS_BLE_HCI                         22
#define RPC_SYS_UNDEF1                          23
#define RPC_SYS_UNDEF2                          24
#define RPC_SYS_UNDEF3                          25
#define RPC_SYS_UNDEF4                          26
#define RPC_SYS_UNDEF5                          27
#define RPC_SYS_UNDEF6                          28
#define RPC_SYS_UNDEF7                          29
#define RPC_SYS_UNDEF8                          30
#define RPC_SYS_SRV_CTRL                        31
  
//! \brief NPI Return Codes
#define NPI_SUCCESS                             0
#define NPI_ROUTING_FULL                        1
#define NPI_SS_NOT_FOUND                        2
#define NPI_INCOMPLETE_PKT                      3
#define NPI_INVALID_PKT                         4
#define NPI_BUSY                                5
#define NPI_TX_MSG_OVERSIZE                     6
#define NPI_TASK_FAILURE                        7 
#define NPI_TASK_INVALID_PARAMS                 8
  
//! \brief Reserved Subsystem ID
#define NPI_SS_RESERVED_ID                      0x00

//! \brief Reserved ICall ID
#define NPI_ICALL_RESERVED_ID                   0x00

//! \brief Masks for cmd0 bits of NPI message
#define NPI_CMD0_TYPE_MASK                      0xE0
#define NPI_CMD0_TYPE_MASK_CLR                  0x1F
#define NPI_CMD0_SS_MASK                        0x1F
#define NPI_CMD0_SS_MASK_CLR                    0xE0
  
#define NPI_SERIAL_TYPE_UART                    0  
#define NPI_SERIAL_TYPE_SPI                     1  // Not supported
  
//! \brief Returns the message type of an NPI message
#define NPI_GET_MSG_TYPE(pMsg)          ((pMsg->cmd0 & NPI_CMD0_TYPE_MASK)>> 5)

//! \brief Returns the subsystem ID of an NPI message
#define NPI_GET_SS_ID(pMsg)             ((pMsg->cmd0) & NPI_CMD0_SS_MASK) 
  
//! \brief Sets the message type of an NPI message
#define NPI_SET_MSG_TYPE(pMsg,TYPE)     pMsg->cmd0 &= NPI_CMD0_TYPE_MASK_CLR; \
                                        pMsg->cmd0 |= ( (TYPE & 0x3) << 5 );

//! \brief Sets the subsystem ID of an NPI message
#define NPI_SET_SS_ID(pMsg,SSID)        pMsg->cmd0 &= NPI_CMD0_SS_MASK_CLR; \
                                        pMsg->cmd0 |= ( (SSID & 0x1F) );

                                        
//! (0x40) Asynchronous Command type for the BLE simple network processor (cmd0 field of npi frame)
#define SNP_NPI_ASYNC_CMD_TYPE    (NPI_MSG_TYPE_ASYNC<<5)

#define SNP_PLC_CONTROL                       0x01
#define SNP_PLC_CONTROL_ACK                   0x81
  
#define SNP_PLC_STATUS_SYNC                   0x02
#define SNP_PLC_STATUS_SYNC_ACK               0x82
  
#define SNP_PLC_STATUS_SEND                   0x03
#define SNP_PLC_STATUS_SEND_ACK               0x83                                        
// ****************************************************************************
// typedefs
// ****************************************************************************

//! \brief Generic message NPI Frame. All messages sent over NPI Transport Layer
//         will follow this structure. Any messages passed between NPI Task and
//         subsystems will be of this type.
typedef struct _npiFrame_t
{
    uint16_t              dataLen;
    uint8_t               cmd0;
    uint8_t               cmd1;
    uint8_t               *pData;
} _npiFrame_t;

typedef union
{
  UART_Params uartParams;
  SPI_Params spiParams;                 // Not supported
} npiInterfaceParams;

typedef enum
{
  HS_INITIATOR,
  HS_RESPONDER,
}hsTransactionRole;

//*****************************************************************************
// globals
//*****************************************************************************
//*****************************************************************************
// function prototypes
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif /* NPI_DATA_H */

