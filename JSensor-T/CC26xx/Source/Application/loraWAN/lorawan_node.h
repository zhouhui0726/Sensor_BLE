/*******************************************************************************
* lorawan_node.h
* modify:  wucong
* date: 20160414
*******************************************************************************/
#ifdef LORA_WAN_ENABLE
#ifndef LORAWAN_NODE_H
#define LORAWAN_NODE_H
#include "Board.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include "util.h"
  
#ifdef __cplusplus
extern "C" {
#endif
/*
typedef struct lorawan_dev_config {
    uint8_t  dev_eui[8];
    uint8_t  app_eui[8];
    uint8_t  app_key[16];
    uint32_t tx_duty_cycle; 
    DeviceClass_t  class;
    bool     adr_enable;
    bool     tx_confirmed;
    uint8_t  default_datarate;
} lorawan_dev_config_t;
*/
/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE1          10000 // 20 [s] value in ms
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE2          300000 // 20 [s] value in ms

/*!
 * Defines the application data transmission duty cycle. 5s, value in [us].
 */
#define APP_TX_DUTYCYCLE                            120000//2400000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [us].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              false

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                            true

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                                1

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_SIZE                           5//4


#define TEST_MODE                                       0
#define NORMAL_MODE                                     1

#define LORAWAN_MAC_STATE_CHECK_EVT                     0x0001
#define LORAWAN_TX_DELAYED_EVT                          0x0002
#define LORAWAN_RX_WINDOW_TIMER1_EVT                    0x0004
#define LORAWAN_RX_WINDOW_TIMER2_EVT                    0x0008
#define LORAWAN_ACK_TIMEOUT_EVT                         0x0010
#define LORAWAN_RX_TIMEOUT_EVT                          0x0020
#define LORAWAN_TX_TIMEOUT_EVT                          0x0040
#define LORAWAN_RX_TIMEOUT_SYNCWORD_EVT                 0x0080
#define LORAWAN_TX_NEXTPACKET_EVT                       0x0100
#define LORAWAN_RX_WINDOW_TIMER3_EVT                    0x0200
#define LORAWAN_RX_WINDOW_TIMER4_EVT                    0x0400
#define LORAWAN_JOIN_NETWORK_EVT                        0x0800


#define SPI_LORA_INT1					0x01
#define SPI_LORA_INT2					0x02
#define SPI_LORA_INT3					0x04

extern Clock_Struct MacStateCheckTimer;
extern Clock_Struct TxDelayedTimer ;
extern Clock_Struct RxWindowTimer1;
extern Clock_Struct RxWindowTimer2 ;
extern Clock_Struct AckTimeoutTimer;
extern Clock_Struct TxTimeoutTimer ;
extern Clock_Struct RxTimeoutTimer;
extern Clock_Struct RxTimeoutSyncWord ;
extern Clock_Struct TxNextPacketTimer;
extern Clock_Struct RxWindowTimer4;
extern Clock_Struct JoinNetworkTimer;

extern void LoRaNetwork_SetAddr(uint8_t* ownAddress);
extern void LoRaNetwork_Init(int flag,uint8_t speed);
extern void JC_BLEPeripheral_LoraProcessIrq(void);
extern void LoRaNetwork_eventsProcess(void);
extern bool LoraWan_SendData(uint8_t IsTxConfirmed,  uint8_t AppPort, uint8_t AppDataSize, uint8_t* AppData);
extern void RetartTxNextPacketTimer(uint8_t IsConfirmed, uint32_t delay);

extern uint32_t lora_enable;


#ifdef __cplusplus
}
#endif

#endif /* LORAWAN_NODE_H */
#endif

