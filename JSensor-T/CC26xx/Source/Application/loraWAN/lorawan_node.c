
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifdef LORA_WAN_ENABLE

#include <string.h>
#include <math.h>
#include "lboard.h"
#include "LoRaMac.h"
#include "Comissioning.h"
#include "lorawan_node.h"
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#ifdef USE_ICALL
  #include <ICall.h>
#endif /* USE_ICALL */

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/PIN/PINCC26XX.h>
#include "Board.h"
#include "ICall.h"
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include "hal_mcu.h"
#include "hal_sleep.h"
#include "osal.h"
#include "util.h"

#include "osal_snv.h"
#include "comdef.h"
#include "bcomdef.h"
#include "JC_GATTprofile.h"
#include "JC_BLEPeripheral.h"

#include "bsp_spi.h"

#include "sx1278-Fsk.h"
#include "temperature_sensor.h"
#include "app_control_service.h"

volatile uint8_t gCurrentLoraRunStat;
volatile uint8_t gLastLoraRunStat;
volatile uint8_t gLoraIsrFlag;

extern void SX1278OnDio0Irq( void );
extern void SX1278OnDio1Irq( void );
extern void SX1278OnDio3Irq( void );
void lorawan_main( void );
void OnTxNextPacketTimerEvent( void );

static void JC_BLEPeripheral_LoraISR_callback(PIN_Handle handle, PIN_Id pinId);
void JC_BLEPeripheral_LoraProcessIrq(void);
void JC_lorawan_clockHandler(UArg arg);


#ifdef POWER_SAVING
//! \brief Flag for low power mode
extern void JC_BLEPeripheral_relPM(void);
extern void JC_BLEPeripheral_setPM(void);
#endif // POWER_SAVING

uint16 iPktLossNum;

extern ICall_Semaphore sem;
static uint16_t lorawan_events;

// Global pin resources
extern PIN_State pinGpioState;
extern PIN_Handle hGpioPin;
uint8 lora_ieee_addr[8];
uint32_t lora_enable = 1;

Clock_Struct MacStateCheckTimer;
Clock_Struct TxDelayedTimer ;
Clock_Struct RxWindowTimer1;
Clock_Struct RxWindowTimer2 ;
Clock_Struct AckTimeoutTimer;
Clock_Struct TxTimeoutTimer ;
Clock_Struct RxTimeoutTimer;
Clock_Struct RxTimeoutSyncWord ;
Clock_Struct TxNextPacketTimer;
Clock_Struct RxWindowTimer4;
Clock_Struct JoinNetworkTimer;


/*********************************************************************
* @fn      LoRaNetwork_SetAddr
*
* @brief   LoRaNetwork_SetAddr 
*
* @return  none
*/
void LoRaNetwork_SetAddr(uint8_t* ownAddress) {
#ifdef TI_DRIVERS_LORA_ENABLE
  // use 6 bytes of device address for 8 bytes of system ID value
  lora_ieee_addr[0] = ownAddress[0];
  lora_ieee_addr[1] = ownAddress[1];
  lora_ieee_addr[2] = ownAddress[2];
  lora_ieee_addr[3] = ownAddress[3];
  lora_ieee_addr[4] = ownAddress[4];
  lora_ieee_addr[5] = ownAddress[5]; 
#ifdef USE_MD5
  lora_ieee_addr[6] = ownAddress[6];
  lora_ieee_addr[7] = ownAddress[7];
#endif
#endif //TI_DRIVERS_LORA_ENABLE
}

/*********************************************************************
* @fn      LoRaNetwork_Init
*
* @brief   LoRaNetwork_Init 
*
* @return  none
*/
void LoRaNetwork_Init(int flag,uint8_t speed) {
#ifdef TI_DRIVERS_LORA_ENABLE
  if(flag == 0)//test mode
  {
    bspSpiOpen();
    System_printf("SX1278FskInit\r\n");
    SX1278FskInit();
  }
  else{//normal mode
    bspSpiOpen();
        
    JC_BLEPeripheral_blinkLed(Board_LED2, 1);
    JC_BLEPeripheral_blinkLed(Board_LED1, 1);

    PIN_registerIntCb(hGpioPin, JC_BLEPeripheral_LoraISR_callback);
    //Enable interrupt
    PIN_setConfig(hGpioPin, PIN_BM_IRQ, Board_LORA_INT1 | PIN_IRQ_POSEDGE);
    PIN_setConfig(hGpioPin, PIN_BM_IRQ, Board_LORA_INT2 | PIN_IRQ_POSEDGE);
    PIN_setConfig(hGpioPin, PIN_BM_IRQ, Board_LORA_INT3 | PIN_IRQ_POSEDGE);
#ifdef POWER_SAVING
    PIN_setConfig(hGpioPin, PINCC26XX_BM_WAKEUP, Board_LORA_INT1 | PINCC26XX_WAKEUP_POSEDGE);
    PIN_setConfig(hGpioPin, PINCC26XX_BM_WAKEUP, Board_LORA_INT2 | PINCC26XX_WAKEUP_POSEDGE);
    PIN_setConfig(hGpioPin, PINCC26XX_BM_WAKEUP, Board_LORA_INT3 | PINCC26XX_WAKEUP_POSEDGE);
#endif // POWER_SAVING

    Util_constructClock(&MacStateCheckTimer, JC_lorawan_clockHandler,
                              MAC_STATE_CHECK_TIMEOUT, 0, false, LORAWAN_MAC_STATE_CHECK_EVT);   
    Util_constructClock(&TxDelayedTimer, JC_lorawan_clockHandler,
                          1000, 0, false, LORAWAN_TX_DELAYED_EVT); 
    Util_constructClock(&RxWindowTimer1, JC_lorawan_clockHandler,
                          1000, 0, false, LORAWAN_RX_WINDOW_TIMER1_EVT); 
    Util_constructClock(&RxWindowTimer2, JC_lorawan_clockHandler,
                          1000, 0, false, LORAWAN_RX_WINDOW_TIMER2_EVT); 
    Util_constructClock(&AckTimeoutTimer, JC_lorawan_clockHandler,
                          1000, 0, false, LORAWAN_ACK_TIMEOUT_EVT);
    Util_constructClock(&TxTimeoutTimer, JC_lorawan_clockHandler,
                          1000, 0, false, LORAWAN_TX_TIMEOUT_EVT); 
    Util_constructClock(&RxTimeoutTimer, JC_lorawan_clockHandler,
                          1000, 0, false, LORAWAN_RX_TIMEOUT_EVT); 
    Util_constructClock(&RxTimeoutSyncWord, JC_lorawan_clockHandler,
                          1000, 0, false, LORAWAN_RX_TIMEOUT_SYNCWORD_EVT);                         
    Util_constructClock(&TxNextPacketTimer, JC_lorawan_clockHandler,
                           500, 0, false, LORAWAN_TX_NEXTPACKET_EVT);                       
    Util_constructClock(&RxWindowTimer4, JC_lorawan_clockHandler,
                          4000, 0, false, LORAWAN_RX_WINDOW_TIMER4_EVT); 
    Util_constructClock(&JoinNetworkTimer, JC_lorawan_clockHandler,
                          3000, 0, false, LORAWAN_JOIN_NETWORK_EVT);     
    Util_restartClock(&JoinNetworkTimer, 3000);
  }

#endif
}

static void JC_lorawan_clockHandler(UArg arg)
{
  // Store the event.
  lorawan_events |= arg;
  
  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
* @fn      LoRaNetwork_eventsProcess
*
* @brief   LoRaNetwork_eventsProcess 
*
* @return  none
*/
void LoRaNetwork_eventsProcess(void) {
#ifdef TI_DRIVERS_LORA_ENABLE
   while (lorawan_events) {
        if (lorawan_events & LORAWAN_MAC_STATE_CHECK_EVT) {
            lorawan_events &= ~LORAWAN_MAC_STATE_CHECK_EVT;
            OnMacStateCheckTimerEvent();
        } 
        if (lorawan_events & LORAWAN_TX_DELAYED_EVT) {
            lorawan_events &= ~LORAWAN_TX_DELAYED_EVT;
            OnTxDelayedTimerEvent();
        } 
        if (lorawan_events & LORAWAN_ACK_TIMEOUT_EVT) {
            lorawan_events &= ~LORAWAN_ACK_TIMEOUT_EVT;
            OnAckTimeoutTimerEvent();
        } 
        if (lorawan_events & LORAWAN_RX_WINDOW_TIMER1_EVT) {
            lorawan_events &= ~LORAWAN_RX_WINDOW_TIMER1_EVT;
            OnRxWindow1TimerEvent();  
        }

        if (lorawan_events & LORAWAN_RX_WINDOW_TIMER4_EVT) {
            lorawan_events &= ~LORAWAN_RX_WINDOW_TIMER4_EVT;  
#ifdef JC_EXPAND            
            recive_timer_time += r4_interval;
            if ((recive_timer_time > 0) && (recive_timer_time > (ICall_getTicks() / 100))) {               
                Util_restartClock(&RxWindowTimer4, recive_timer_time - (ICall_getTicks() / 100)); 
            } else {
                Util_restartClock(&RxWindowTimer4, r4_interval); 
            }
            if ((ICall_getTicks() / 100) - rx1_window_time > MAX_RX_WINDOW) {
                OnRxWindow4TimerEvent();   
            }
#endif
        }
        if (lorawan_events & LORAWAN_RX_WINDOW_TIMER2_EVT) {
            lorawan_events &= ~LORAWAN_RX_WINDOW_TIMER2_EVT;
            OnRxWindow2TimerEvent();
        }
        if (lorawan_events & LORAWAN_RX_TIMEOUT_EVT) {
            lorawan_events &= ~LORAWAN_RX_TIMEOUT_EVT;
            SX1278OnTimeoutIrq();
        }
        if (lorawan_events & LORAWAN_TX_TIMEOUT_EVT) {
            lorawan_events &= ~LORAWAN_TX_TIMEOUT_EVT;           
            SX1278OnTimeoutIrq();
        }
        if (lorawan_events & LORAWAN_RX_TIMEOUT_SYNCWORD_EVT) {
            lorawan_events &= ~LORAWAN_RX_TIMEOUT_SYNCWORD_EVT;          
            SX1278OnTimeoutIrq();
        }
        if (lorawan_events & LORAWAN_TX_NEXTPACKET_EVT) {
            lorawan_events &= ~LORAWAN_TX_NEXTPACKET_EVT;
            OnTxNextPacketTimerEvent();      
        } 
        if (lorawan_events & LORAWAN_JOIN_NETWORK_EVT) {
            lorawan_events &= ~LORAWAN_JOIN_NETWORK_EVT;            
            lorawan_main();                 
        }
   }
#endif //TI_DRIVERS_LORA_ENABLE
}

/*!*****************************************************************************
*  @fn         JC_BLEPeripheral_Lora_callback
*
*  Interrupt service routine for buttons, relay and MPU
*
*  @param      handle PIN_Handle connected to the callback
*
*  @param      pinId  PIN_Id of the DIO triggering the callback
*
*  @return     none
******************************************************************************/
static void JC_BLEPeripheral_LoraISR_callback(PIN_Handle handle, PIN_Id pinId)
{
  switch (pinId) {
    
  case Board_LORA_INT1:
    gLoraIsrFlag |= SPI_LORA_INT1;
    Semaphore_post(sem);//Generate a signal to wake up the thread,Thread to handle  task;
    break;
    
  case Board_LORA_INT2:
    gLoraIsrFlag |= SPI_LORA_INT2;
    Semaphore_post(sem);
    break;
    
  case Board_LORA_INT3:
    gLoraIsrFlag |= SPI_LORA_INT3;
    Semaphore_post(sem);
    break;
  default:
    /* Do nothing */
    break;
  }
#ifdef POWER_SAVING
  if (gLoraIsrFlag)
  {
    JC_BLEPeripheral_setPM();
  }
#endif //POWER_SAVING
}

void JC_BLEPeripheral_LoraProcessIrq(void)
{  
  
  if (!gLoraIsrFlag) {
      return;
  }
  if (gLoraIsrFlag & SPI_LORA_INT1) {   
    gLoraIsrFlag &= ~SPI_LORA_INT1;
    SX1278OnDio0Irq();      
    JC_BLEPeripheral_blinkLed(Board_LED1, 1);
  } else if (gLoraIsrFlag & SPI_LORA_INT2) {    
     gLoraIsrFlag &= ~SPI_LORA_INT2;
     SX1278OnDio1Irq();     
  } else if (gLoraIsrFlag & SPI_LORA_INT3) {         
    gLoraIsrFlag &= ~(SPI_LORA_INT1 + SPI_LORA_INT2 + SPI_LORA_INT3);
    SX1278OnDio3Irq();    
  } 
#ifdef POWER_SAVING
  JC_BLEPeripheral_relPM();
#endif //POWER_SAVING
}

#if( OVER_THE_AIR_ACTIVATION != 0 )

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#else

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

static uint32_t ReportDutyCycleTime;
/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;
/*!
 * Device states
 */
enum eDevicState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    dht11_data_t dht11_value;
    switch( port )
    {
    case 1:
        {
            AppData[0] = 0x3;           
            AppData[1] = 0x3;   
            getTsensorValue(&dht11_value);
            AppData[2] = dht11_value.temp_value;
            AppData[3] = dht11_value.humi_value;
            AppData[4] = Get_BattMeasure();
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = DR_0;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = DR_0;
            IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = DR_0;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

void lorawan_main( void );


/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

 //   TimerStop( &TxNextPacketTimer );
//    Util_stopClock( &TxNextPacketTimer );
    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );
    
   if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
            DeviceState = DEVICE_STATE_INIT;
        }
    }
    lorawan_main();
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
void RetartTxNextPacketTimer(uint8_t IsConfirmed, uint32_t delay)
{   
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );
    
    if( status == LORAMAC_STATUS_OK ) {        
        if( mibReq.Param.IsNetworkJoined == true ) {                         
            IsTxConfirmed = IsConfirmed;  
            Util_restartClock(&TxNextPacketTimer, delay); 
        }  
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
bool LoraWan_SendData(uint8_t IsTxConfirmed, uint8_t AppPort, uint8_t AppDataSize,  uint8_t* AppData)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = DR_0;
    }
    else
    {
        if( IsTxConfirmed == MCPS_UNCONFIRMED )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = DR_0;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = DR_0;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *McpsConfirm )
{

    if( McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( McpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbRetries
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
//        GpioWrite( &Led1, 0 );

    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] McpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *McpsIndication )
{
    uint8_t i;
    uint16_t temp;

    if( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }    
                            
    switch( McpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            if (McpsIndication->Port == LORAWAN_APP_PORT) {
 //               memcpy(AppData, 0, 4);
 //               LoraWan_SendData(MCPS_UNCONFIRMED, LORAWAN_APP_PORT, 4, AppData);
            }
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( McpsIndication->RxData == true )
    {
//        System_printf("McpsIndication %d B :", McpsIndication->BufferSize);
        for (i = 0; i < McpsIndication->BufferSize; i++) {
//            System_printf(" %02x", McpsIndication->Buffer[i]);
        }       
//        System_printf("\r\n");
        
        switch( McpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
            
            if( McpsIndication->BufferSize == 5 ) {
                switch (McpsIndication->Buffer[0]) {
                    case 1:  
                        App_Control_Opera_Service(1);  
                        break;
                    case 2:                     
                        App_Control_Opera_Service(2);
                        break;
                    case 4:
                        if (lora_enable) {
                             RetartTxNextPacketTimer(true, 4000);
                        }
                        break;
                    case 5:
                        temp = McpsIndication->Buffer[1] * 256;
                        temp += McpsIndication->Buffer[2];
                        if (temp != 0) {
                            ReportDutyCycleTime = temp * 60;
                        }                                              
                }
            }

            break;
    
        default:
            break;
        }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *MlmeConfirm )
{
    if( MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( MlmeConfirm->MlmeRequest )
        {
            case MLME_JOIN:
            {
                // Status is OK, node has joined the network
                break;
            }
            case MLME_LINK_CHECK:
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = MlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = MlmeConfirm->NbGateways;
                }
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
}

static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t   LoRaMacCallbacks;
static MibRequestConfirm_t mibReq; 

#pragma optimize=none  
/**
 * Main application entry point.
 */
void lorawan_main( void )
{     
    static uint32_t  LoRaMacJoinCount = 0;
    
    switch( DeviceState )
    {
        case DEVICE_STATE_INIT:
        {
            LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
            LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
            LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
            LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
            
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );           
            if (!lora_enable) {                  
                return;
            }

            mibReq.Type = MIB_ADR;
            mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_PUBLIC_NETWORK;
            mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            mibReq.Type = MIB_DEVICE_CLASS;
            mibReq.Param.Class = CLASS_A;
            LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
            LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
            DeviceState = DEVICE_STATE_JOIN;
//                break;
        }
        case DEVICE_STATE_JOIN:
        {
#if( OVER_THE_AIR_ACTIVATION != 0 )
            MlmeReq_t mlmeReq;
            // Initialize LoRaMac device unique ID
            BoardGetUniqueId( DevEui );

            if ((DevEui[0] == 0) && (DevEui[1] == 0) &&(DevEui[2] == 0) &&(DevEui[3] == 0) && 
                (DevEui[4] == 0) && (DevEui[5] == 0) &&(DevEui[6] == 0) &&(DevEui[7] == 0)) {
                NextTx = false;
                return;            
            }
            
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;
            if( NextTx == true )
            {                   
                 LoRaMacMlmeRequest( &mlmeReq );               
            }
            if (LoRaMacJoinCount++ < 100) {
                // Schedule next packet transmission
                TxDutyCycleTime = OVER_THE_AIR_ACTIVATION_DUTYCYCLE1;
            } else {
                TxDutyCycleTime = OVER_THE_AIR_ACTIVATION_DUTYCYCLE2;
            }
            Util_restartClock(&TxNextPacketTimer, TxDutyCycleTime);

            DeviceState = DEVICE_STATE_SLEEP;
#else
            // Random seed initialization
            srand1( BoardGetRandomSeed( ) );

            // Choose a random device address
            DevAddr = randr( 0, 0x01FFFFFF );

            mibReq.Type = MIB_NET_ID;
            mibReq.Param.NetID = LORAWAN_NETWORK_ID;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_DEV_ADDR;
            mibReq.Param.DevAddr = DevAddr;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_NWK_SKEY;
            mibReq.Param.NwkSKey = NwkSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_APP_SKEY;
            mibReq.Param.AppSKey = AppSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mibReq.Type = MIB_NETWORK_JOINED;
            mibReq.Param.IsNetworkJoined = true;
            LoRaMacMibSetRequestConfirm( &mibReq );

            DeviceState = DEVICE_STATE_SEND;
#endif
            break;
        }
        case DEVICE_STATE_SEND:
        {
            LoRaMacJoinCount = 0;
            if( NextTx == true )
            {                    
//                System_printf("DEVICE_STATE_SEND %d\r\n", ICall_getTicks() / 100);
                PrepareTxFrame( AppPort );
                NextTx = SendFrame( );               
            }
            if( ComplianceTest.Running == true )
            {
                // Schedule next packet transmission as soon as possible
                TxDutyCycleTime = 300000; // 1 ms
            }

            // Schedule next packet transmission
            if (ReportDutyCycleTime != 0) {
                TxDutyCycleTime = ReportDutyCycleTime + randr(0, APP_TX_DUTYCYCLE );
            } else {
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr(0, APP_TX_DUTYCYCLE );
            }
            Util_restartClock(&TxNextPacketTimer, TxDutyCycleTime);                

            DeviceState = DEVICE_STATE_SLEEP;
            break;
        }
        
        case DEVICE_STATE_SLEEP:
        {                
            break;
        }
        default:
        {
            DeviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}

#endif
