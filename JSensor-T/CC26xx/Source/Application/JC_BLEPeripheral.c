/*************************************************************************************************
Filename:       JC_BLEPeripheral.c
Revised:        $Date: 2015-07-13 11:43:11 -0700 (Mon, 13 Jul 2015) $
Revision:       $Revision: 44336 $

Description:    This file contains the JC BLE Peripheral sample application
for use with the CC2650 Bluetooth Low Energy Protocol Stack.

Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

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
PROVIDED 鎻係 IS锟絎ITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/System.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "AskCrc.h"
#include "JC_GATTprofile.h"
#include "Board.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "JC_peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_lcd.h"
#include "Board.h"

#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
#include "tl.h"

#include "npi_task_swhs.h"
#include "npi_data_swhs.h"                    

#define SNP_PLC_EVT                           0x0100
#define SNP_NEW_AP_MSG_EVT                    0x0200
static Clock_Struct SNP_PLC_Clock;  

#define NPI_TIMEOUT_EVT                       0x0400

//events that TL will use to control the driver
#define MRDY_EVENT                            0x0010
#define TRANSPORT_RX_EVENT                    0x0020
#define TRANSPORT_TX_DONE_EVENT               0x0040

#define APP_TL_BUFF_SIZE                      220

static uint8_t appRxBuf[APP_TL_BUFF_SIZE];

static void JC_BLEPeripheral_TLpacketParser(void);

static TLCBs_t JC_BLEPeripheral_TLCBs =
{
  JC_BLEPeripheral_TLpacketParser // parse data read from transport layer
};

Clock_Struct NPI_Timeout_Clock;  
#endif //TL

#ifdef USE_PTM
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>

static PIN_Config TestModePinsCfg[] =
{
  TEST_PIN | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,     /* LED initially off  */
  Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,     /* LED initially off */
  PIN_TERMINATE
};

static PIN_State TestModepinGpioState;
static PIN_Handle TestModehGpioPin;

#endif //USE_PTM

#include "JC_BLEPeripheral.h"

#ifdef UART_PRINT_ENABLE
#include "uart_printf.h"
#endif

#ifdef TI_DRIVERS_LORA_ENABLE
#ifdef LORA_WAN_ENABLE
#include "lorawan_node.h"
#endif //Lora
#endif

#ifdef APP_HANDSHAKE_ENABLE
#include "app_handshake_service.h"
#endif  // APP_HANDSHAKE_ENABLE

#ifdef APP_CONTROL_ENABLE
#include "app_control_service.h"
#endif

#include "temperature_sensor.h"

static Clock_Struct SendStatToBleClock;
#ifdef APP_CONTROL_ENABLE
static void JC_SendStatusToBLE(void);
#endif
static void Process_Ask_and_Lora_Status(uint8_t stat);

static Clock_Struct UpdateSensorDataClock;
dht11_data_t dht11_value;


/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} sbpEvt_t;


/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
ICall_Semaphore sem;

// Clock instances for internal periodic events.
Clock_Struct periodicClock;

//batt Collection time
static Clock_Struct BattCaptureClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;



#ifdef POWER_SAVING
//! \brief Flag for low power mode
static volatile bool JC_BLEPeripheralPMSetConstraint = FALSE;
void JC_BLEPeripheral_relPM(void);
void JC_BLEPeripheral_setPM(void);
#endif // POWER_SAVING

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
uint8_t scanRspData[] =
{
  0x0A,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'J',   // 'J'
  'S',   // 'C'
  'e',   // 'l'
  'n',   // 'o'
  's',   // 'c'
  'o',   // 'k'
  'n',   // 'e'
  '-',   
  'T',   
  /*
  */
  //  0x03,   // length of this data
  //  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  //  0xF0,
  //  0xFF,
  0x0C,
  GAP_ADTYPE_SERVICE_DATA,
  0x0A,
  0x18,
  
  0x00,//6 byte mac addr
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  
  0x00,
  0x64,      // 100% default
  0x20	     //temp 20 C
};
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_COMPLETE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#else
  LO_UINT16(SCAN_RSP_SERV_UUID),
  HI_UINT16(SCAN_RSP_SERV_UUID),
#endif //!FEATURE_OAD
  0x05,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  'V',
  '1',
  '.',
  '0' 
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN];

uint8_t UserCharVersion[VER_LEN] = {'1', '3'};
//version 1.3 
//1.2 optimize task code 
//1.2 modification ADC code
//1.3 modification RF add PIC process
uint8_t UserCharUUID[ATT_UUID_SIZE] =
{
  0xF0, 0xF0, 0xC1, 0xC1,
  0x00, 0x01, 0x00, 0x00,
  0x00, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF
};
uint8_t UserCharMMID[MAJOR_MINOR_LEN] =
{
  0xFF,
  0xFF,
  0xFF,
  0xFF
};
uint8_t UserCharAdv[ADVERTISING_INTERVAL_LEN] =
{
  0x01,
  0x90
};
uint8_t UserCharPower[POWER_TX_LEN] =
{
  0x0C,
  0xC5
};
uint8_t UserAdjusting[4] =
{
  1,
  2,
  3,
  4
};

uint8_t UserCharAppPwd[16] =
{
  0x31, 0x32, 0x33, 0x34, 
  0x35, 0x36, 0x31, 0x32, 
  0x33, 0x34, 0x35, 0x36,
  0x31, 0x32, 0x33, 0x34
};
uint8_t UserCharAppFlag = 0;

uint8_t module_Ask_and_Lora_status;

#ifdef USE_MD5

#include "md5_new.h"
#define DEVICE_NAME "JSensor-T"
char Md5_serial_id_and_name[16] = {0};
uint8_t Serial_number[9];
void JC_BLEPeripheral_Get_md5(uint8_t *serial_number, uint8_t len);
#endif // USE_MD5


// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

//static bool scanningStarted = FALSE;
/*********************************************************************
* LOCAL FUNCTIONS
*/
static void JC_BLEPeripheral_init( void );
static void JC_BLE_Advertising_Content_init(void);
static void JC_BLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t JC_BLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t JC_BLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void JC_BLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void JC_BLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void JC_BLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void JC_BLEPeripheral_performPeriodicTask(void);

static void JC_BLEPeripheral_sendAttRsp(void);
static void JC_BLEPeripheral_freeAttRsp(uint8_t status);

static void JC_BLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void JC_BLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
static void JC_BLEPeripheral_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData);

#ifdef FEATURE_OAD
void JC_BLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                        uint8_t *pData);
#endif //FEATURE_OAD

static void JC_BLEPeripheral_clockHandler(UArg arg);

#ifdef PLUS_OBSERVER
static void JC_BLEPeripheral_ObserverStateChangeCB(gapPeripheralObserverRoleEvent_t *pEvent);
#endif

void JC_BLEPeripheral_blinkLed(uint8_t led, uint8_t nBlinks);

static void JC_BLEPeripheral_BattMeasure(void);

/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t JC_BLEPeripheral_gapRoleCBs =
{
  JC_BLEPeripheral_stateChangeCB     // Profile State Change Callbacks
#ifdef PLUS_OBSERVER  
    ,JC_BLEPeripheral_ObserverStateChangeCB
#endif
};

// GAP Bond Manager Callbacks
static gapBondCBs_t JC_BLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// JC GATT Profile Callbacks
#ifndef FEATURE_OAD
static JC_ProfileCBs_t JC_BLEPeripheral_JCProfileCBs =
{
  JC_BLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t JC_BLEPeripheral_oadCBs =
{
  JC_BLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      JC_BLEPeripheral_createTask
*
* @brief   Task creation function for the JC BLE Peripheral.
*
* @param   None.
*
* @return  None.
*/
void JC_BLEPeripheral_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;
  
  Task_construct(&sbpTask, JC_BLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      JC_BLE_Advertising_Content_init
*
* @param   None.
*
* @return  None.
*/
static void JC_BLE_Advertising_Content_init(void)
{
  //read flash Initialize the information
  uint8_t UserAdjustingBuff[ADJUST_LEN];
  VOID osal_snv_read(BLE_NVID_USER_SET_ADJUST, ADJUST_LEN, UserAdjustingBuff);
  if (!memcmp(UserAdjustingBuff, UserAdjusting, ADJUST_LEN))
  {
    VOID osal_snv_read(BLE_NVID_USER_SET_UUID, ATT_UUID_SIZE, UserCharUUID);//UUID
    VOID osal_snv_read(BLE_NVID_USER_SET_MMID, MAJOR_MINOR_LEN, UserCharMMID);//major and minor
    VOID osal_snv_read(BLE_NVID_USER_SET_ADV_INT, ADVERTISING_INTERVAL_LEN, UserCharAdv);// advertising interval
    VOID osal_snv_read(BLE_NVID_USER_SET_POWER, POWER_TX_LEN, UserCharPower);//power val
    
    VOID osal_snv_read(BLE_NVID_USER_SET_APP_PWD, 16, UserCharAppPwd);//app
    VOID osal_snv_read(BLE_NVID_USER_SET_APP_FLAG, 1, &UserCharAppFlag);//app
    VOID osal_snv_read(BLE_NVID_USER_SET_LORA_AND_ASK, 1, &module_Ask_and_Lora_status);
    Process_Ask_and_Lora_Status(module_Ask_and_Lora_status);
    
  }
  else
  {
    VOID osal_snv_write(BLE_NVID_USER_SET_UUID, ATT_UUID_SIZE, UserCharUUID);//UUID
    VOID osal_snv_write(BLE_NVID_USER_SET_MMID, MAJOR_MINOR_LEN, UserCharMMID);//major and minor
    VOID osal_snv_write(BLE_NVID_USER_SET_ADV_INT, ADVERTISING_INTERVAL_LEN, UserCharAdv);// advertising interval
    VOID osal_snv_write(BLE_NVID_USER_SET_POWER, POWER_TX_LEN, UserCharPower);//power val
    VOID osal_snv_write(BLE_NVID_USER_SET_ADJUST, ADJUST_LEN, UserAdjusting);//adjusting
    
    VOID osal_snv_write(BLE_NVID_USER_SET_APP_PWD, 16, UserCharAppPwd);//app
    VOID osal_snv_write(BLE_NVID_USER_SET_APP_FLAG, 1, &UserCharAppFlag);//app
    module_Ask_and_Lora_status = 0x02; //default all open
    VOID osal_snv_write(BLE_NVID_USER_SET_LORA_AND_ASK, 1, &module_Ask_and_Lora_status);
    Process_Ask_and_Lora_Status(module_Ask_and_Lora_status);
    
  }
  HCI_EXT_SetTxPowerCmd(UserCharPower[0]);//0~12 :-21dbm to +5dbm, 0 -- -21dbm default
  uint8 bdAddress[B_ADDR_LEN];
#ifdef USE_MD5
  uint8_t cnt = 0;
  while (osal_snv_read(BLE_NVID_USER_SERIAL_NUMBER, sizeof(Serial_number), Serial_number) != SUCCESS)
  {
    delay_ms(100);
    if (cnt++ > 3) {
      break;
    }
  }
  if (Serial_number[sizeof(Serial_number) - 1] == CRC8_Generate(Serial_number, sizeof(Serial_number) - 1)) {
    // Hard code the BD Address till CC2650 board gets its own IEEE address
    for (int i = B_ADDR_LEN; i > 0; i--)
    {
      bdAddress[B_ADDR_LEN - i] = Serial_number[i+1];
    }
    
    HCI_EXT_SetBDADDRCmd(bdAddress);//set user mac addr
    memcpy(bdAddress, &Serial_number[2], B_ADDR_LEN);
    JC_BLEPeripheral_Get_md5(Serial_number, sizeof(Serial_number) - 1);
    memcpy(UserCharAppPwd, Md5_serial_id_and_name, sizeof(Md5_serial_id_and_name));
  } else {
    uint8_t addr[B_ADDR_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, bdAddress);
    memcpy(addr, bdAddress, B_ADDR_LEN);
    for (int i = B_ADDR_LEN; i > 0; i--)
    {
      bdAddress[B_ADDR_LEN - i] = addr[i-1];
    }
  }
#ifdef LORA_WAN_ENABLE
  LoRaNetwork_SetAddr(Serial_number);
#endif
  
#else
  uint8_t addr[B_ADDR_LEN];
  GAPRole_GetParameter(GAPROLE_BD_ADDR, bdAddress);
  memcpy(addr, bdAddress, B_ADDR_LEN);
  for (int i = B_ADDR_LEN; i > 0; i--)
  {
    bdAddress[B_ADDR_LEN - i] = addr[i-1];
  }
#endif
  VOID memcpy(&scanRspData[sizeof(scanRspData) - 9], bdAddress, B_ADDR_LEN);
  

  scanRspData[sizeof(scanRspData) - 3] = UserCharAppFlag;
  VOID memcpy(attDeviceName, &scanRspData[2], 7);
}


/*********************************************************************
* @fn      JC_BLEPeripheral_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void JC_BLEPeripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
  
  // Set device's Sleep Clock Accuracy
#ifdef USE_PTM
  TestModehGpioPin = PIN_open(&TestModepinGpioState, TestModePinsCfg);
  uint8 flag = 0;
  flag = PIN_getInputValue(TEST_PIN);
#if defined (NPI_USE_UART) || defined (NPI_USE_SPI)
  PTM_Uart_enable = false;
#endif
  if (!flag) //enter PTM
  {
    PIN_setOutputValue(TestModehGpioPin, Board_LED1, Board_LED_ON);
    PIN_setOutputValue(TestModehGpioPin, Board_LED2, Board_LED_ON);
    //prevent PM
    Power_setConstraint(Power_SB_DISALLOW);
    Power_setConstraint(Power_IDLE_PD_DISALLOW);
    
#if defined( LORA_MSPAN_ENABLE ) || defined( LORA_JCNETPAN_ENABLE ) || defined( LORA_WAN_ENABLE )
#ifdef TI_DRIVERS_LORA_ENABLE
    LoRaNetwork_Init(TEST_MODE,0);
#endif
#endif
#if defined (NPI_USE_UART) || defined (NPI_USE_SPI)
    //initialize and pass information to TL
    PTM_Uart_enable = true;
    TLinit(&sem, &JC_BLEPeripheral_TLCBs, TRANSPORT_TX_DONE_EVENT,
           TRANSPORT_RX_EVENT, MRDY_EVENT);
#endif //TL
    //enable PTM
#ifdef PTM
    HCI_EXT_EnablePTMCmd();
#else 
    //HCI_EXT_SetTxPowerCmd(9);//0~12 :-21dbm -- +5dbm, 7 -- 0dbm default
    HCI_EXT_ModemTestTxCmd(HCI_EXT_TX_UNMODULATED_CARRIER, 19);//19--2440Mhz
#endif
  }
  else  //proceed as normal
  {
    PIN_close(TestModehGpioPin);
#else //USE_PTM
    {
#endif //!USE_PTM
      hGpioPin = PIN_open(&pinGpioState, BoardGpioInitTable);
      
      //Initialize the Advertising Content
      JC_BLE_Advertising_Content_init();
      
#if defined( LORA_MSPAN_ENABLE ) || defined( LORA_JCNETPAN_ENABLE ) || defined( LORA_WAN_ENABLE )
#ifdef TI_DRIVERS_LORA_ENABLE
      //loranetwork init
      LoRaNetwork_Init(NORMAL_MODE,0);
#endif //TI_DRIVERS_LORA_ENABLE
#endif
      
      // Create an RTOS queue for message from profile to be sent to app.
      appMsgQueue = Util_constructQueue(&appMsg);
      
      // Create one-shot clocks for internal periodic events.
      Util_constructClock(&periodicClock, JC_BLEPeripheral_clockHandler,
                          SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);
      
      Util_constructClock(&SendStatToBleClock, JC_BLEPeripheral_clockHandler,
                          1000, 0, true, SEND_STAT_TO_BLE_EVT);
      
      advertData[sizeof(advertData) - 3] = UserCharVersion[0];
      advertData[sizeof(advertData) - 1] = UserCharVersion[1];

      
      /*****************  OBSERVER  *****************/
#ifdef PLUS_OBSERVER
      
      //Setup GAP Observer params
      {
        uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
        
        GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t),
                             &scanRes);
        
        // Set the GAP Characteristics
        GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION); //how long to scan (in scan state)
        GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
        
        //Set scan interval
        GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, 1600); //every sec 1600*.625=1000ms
        
        //Set scan window
        GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, 320); //every 200ms
        
      }
#endif
      
      // Setup the GAP
      /*****************  PERIPHERAL  *****************/
      GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
      // Setup the GAP Peripheral Role Profile
      {
        // For all hardware platforms, device starts advertising upon initialization
        uint8_t initialAdvertEnable = TRUE;
        
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16_t advertOffTime = 0;
        
        uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        
        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &initialAdvertEnable);
        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                             &advertOffTime);
        
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                             scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        
        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                             &enableUpdateRequest);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMinInterval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMaxInterval);
        GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                             &desiredSlaveLatency);
        GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                             &desiredConnTimeout);
      }
      
      // Set the GAP Characteristics
      GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
      
      // Set advertising interval
      {
        uint16_t advInt = (BUILD_UINT16(UserCharAdv[1], UserCharAdv[0]) * DEFAULT_ADVERTISING_INTERVAL) / 100;//100ms UserCharAdv
        
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt); 
        GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt); 
      }
      
      // Setup the GAP Bond Manager
      {
        uint32_t passkey = 123456; // passkey "000000"
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t mitm = TRUE;
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        uint8_t bonding = FALSE;//no bonding for IOS
        
        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                                &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
      }
      
      // Initialize GATT attributes
      GGS_AddService(GATT_ALL_SERVICES);            // GAP
      GATTServApp_AddService(GATT_ALL_SERVICES);    // GATT attributes
      
      /****************  Customer Profile *********************/
      //#ifndef FEATURE_OAD
      //JC_Profile_AddService(GATT_ALL_SERVICES); // JC_ GATT Profile
      //#endif //!FEATURE_OAD
#ifdef APP_HANDSHAKE_ENABLE
      JC_App_handshake_Profile_AddService();
	  
      App_handshake_Profile_RegisterAppCBs(&JC_BLEPeripheral_JCProfileCBs);
#endif //APP_HANDSHAKE_ENABLE
      /****************  Customer  Profile End *****************/
      
#ifdef FEATURE_OAD
      VOID OAD_addService();                 // OAD Profile
      OAD_register((oadTargetCBs_t *)&JC_BLEPeripheral_oadCBs);
      hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD
      
#ifdef IMAGE_INVALIDATE
      Reset_addService();
#endif //IMAGE_INVALIDATE
      
#if 0      
      //#ifndef FEATURE_OAD
      // Setup the JC_Profile Characteristic Values
      {

        JC_Profile_SetParameter(JC_PROFILE_CHAR1, VER_LEN, UserCharVersion);
        JC_Profile_SetParameter(JC_PROFILE_CHAR2, ATT_UUID_SIZE, UserCharUUID);
        JC_Profile_SetParameter(JC_PROFILE_CHAR3, MAJOR_MINOR_LEN, UserCharMMID);
        JC_Profile_SetParameter(JC_PROFILE_CHAR4, ADVERTISING_INTERVAL_LEN, UserCharAdv);
        JC_Profile_SetParameter(JC_PROFILE_CHAR5, POWER_TX_LEN, UserCharPower);

      }
      // Register callback with JCGATTprofile
      JC_Profile_RegisterAppCBs(&JC_BLEPeripheral_JCProfileCBs);
//#endif //!FEATURE_OAD
#endif
      
      // Start the Device
      VOID GAPRole_StartDevice(&JC_BLEPeripheral_gapRoleCBs);
      
      // Start Bond Manager
      VOID GAPBondMgr_Register(&JC_BLEPeripheral_BondMgrCBs);
      
      // Register with GAP for HCI/Host messages
      GAP_RegisterForMsgs(selfEntity);
      
      // Register for GATT local events and ATT Responses pending for transmission
      GATT_RegisterForMsgs(selfEntity);
      	
      /*****************  Battery Monitor Start  ***************/
    
      // Enable the Battery Monitor.
      AONBatMonEnable();
      // measurement results are only updated if changes occur.
      JC_BLEPeripheral_BattMeasure();//batt
      Util_constructClock(&BattCaptureClock, JC_BLEPeripheral_clockHandler,
                        300000, 0, false, SBP_BATT_CAP_EVT);
      Util_startClock(&BattCaptureClock);// 5 minutes to collect the battery 
    
    /****************  Battery Monitor End  *****************/
    
      
      temperatureSensorInit();
      
      Util_constructClock(&UpdateSensorDataClock, JC_BLEPeripheral_clockHandler,
                          3000, 0, true, UPDATE_SENSOR_DATA_EVT);
      
#ifdef APP_CONTROL_ENABLE
      App_Control_Profile_AddService();
      
      App_Control_Profile_RegisterAppCBs(&JC_BLEPeripheral_JCProfileCBs);
      
      App_Control_Profile_SetParameter(APP_CONTROL_PROFILE_CHAR4, sizeof(uint8_t), &scanRspData[sizeof(scanRspData) - 2]); //send battery
#endif
      
#ifdef UART_PRINT_ENABLE			
      printS("Device Type: BLE Peripheral And Observer.");
      printS("SYS START...!");
#endif //UART_PRINT_ENABLE
    }
  }
  
  /*********************************************************************
  * @fn      JC_BLEPeripheral_taskFxn
  *
  * @brief   Application task entry point for the JC BLE Peripheral.
  *
  * @param   a0, a1 - not used.
  *
  * @return  None.
  */
  static void JC_BLEPeripheral_taskFxn(UArg a0, UArg a1)
  {
    // Initialize application
    JC_BLEPeripheral_init();
    // Application main loop
    for (;;)
    {
      // Waits for a signal to the semaphore associated with the calling thread.
      // Note that the semaphore associated with a thread is signaled when a
      // message is queued to the message receive queue of the thread or when
      // ICall_signal() function is called onto the semaphore.
      ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);
      
#ifdef TI_DRIVERS_LORA_ENABLE     
#if defined( LORA_WAN_ENABLE )     
      JC_BLEPeripheral_LoraProcessIrq();
      LoRaNetwork_eventsProcess();
#endif
#endif //TI_DRIVERS_LORA_ENABLE
      
#if defined (NPI_USE_UART) || defined (NPI_USE_SPI)     
      //TL handles driver events. this must be done first
      if (PTM_Uart_enable) {
        TL_handleISRevent();
      } else {
#endif //TL
      
      if (errno == ICALL_ERRNO_SUCCESS)
      {
        ICall_EntityID dest;
        ICall_ServiceEnum src;
        ICall_HciExtEvt *pMsg = NULL;
        
        if (ICall_fetchServiceMsg(&src, &dest,
                                  (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
        {
          uint8 safeToDealloc = TRUE;
          
          if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
          {
            ICall_Event *pEvt = (ICall_Event *)pMsg;
            
            // Check for BLE stack events first
            if (pEvt->signature == 0xffff)
            {
              if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
              {
                // Try to retransmit pending ATT Response (if any)
                JC_BLEPeripheral_sendAttRsp();
              }
            }
            else
            {
              // Process inter-task message
              safeToDealloc = JC_BLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
            }
          }
          
          if (pMsg && safeToDealloc)
          {
            ICall_freeMsg(pMsg);
          }
        }
        
        // If RTOS queue is not empty, process app message.
        while (!Queue_empty(appMsgQueue))
        {
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            JC_BLEPeripheral_processAppMsg(pMsg);
            
            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
#if defined (NPI_USE_UART) || defined (NPI_USE_SPI)    
    }
#endif
      //handshake event
      if (events & SBP_PERIODIC_EVT)
      {
        events &= ~SBP_PERIODIC_EVT;
        //Util_startClock(&periodicClock);
        JC_BLEPeripheral_performPeriodicTask();
      }
      
      if (events & SBP_BATT_CAP_EVT)
      {
        events &= ~SBP_BATT_CAP_EVT;
        JC_BLEPeripheral_BattMeasure();//Measure batt and temp
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                             scanRspData);//updata scan rsp data for 5min times
        Util_startClock(&BattCaptureClock);
      }
       
#ifdef APP_CONTROL_ENABLE
      if (events & SEND_STAT_TO_BLE_EVT) 
      {
        events &= ~SEND_STAT_TO_BLE_EVT;
        JC_SendStatusToBLE();
        Util_startClock(&SendStatToBleClock);
      }
#endif
      
      if(events & UPDATE_SENSOR_DATA_EVT)
      {
        events &= ~UPDATE_SENSOR_DATA_EVT;
        getTemperature(&dht11_value);
        Util_startClock(&UpdateSensorDataClock);
        
      }
      
#ifdef FEATURE_OAD
      while (!Queue_empty(hOadQ))
      {
        oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);
        
        // Identify new image.
        if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
        {
          OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
        }
        // Write a next block request.
        else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
        {
          OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
        }
        
        // Free buffer.
        ICall_free(oadWriteEvt);
      }
#endif //FEATURE_OAD
    }
}
  
#ifdef APP_CONTROL_ENABLE
static void JC_SendStatusToBLE(void)
{
  //App_Control_Profile_SetParameter(APP_CONTROL_PROFILE_CHAR2, sizeof(uint8_t), &g_Idle_Stat);
}
#endif

#ifdef PLUS_OBSERVER  
/*********************************************************************
* @fn      JC_BLECentral_processRoleEvent
*
* @brief   Central role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void JC_BLEPeripheralObserver_processRoleEvent(gapPeripheralObserverRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    //    case GAP_DEVICE_INIT_DONE_EVENT:
    //      {
    //        LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->initDone.devAddr),
    //                         LCD_PAGE1);
    //        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
    //      }
    //      break;
    
  case GAP_DEVICE_INFO_EVENT:
    {
      int8 Phone_Rssi = 0;
      uint8 addr[B_ADDR_LEN];
      //LCD_WRITE_STRING("GAP_DEVICE_INFO_EVENT", LCD_PAGE6);
      Phone_Rssi = pEvent->deviceInfo.rssi;
      dummy = pEvent->deviceInfo.dataLen;
      LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->deviceInfo.addr), LCD_PAGE6);
    }
    break;
    
  case GAP_DEVICE_DISCOVERY_EVENT:
    {
      // discovery complete
      //scanningStarted = FALSE;
      LCD_WRITE_STRING("GAP_DEVICE_DISC_EVENT", LCD_PAGE7);
    }
    break;
    
  default:
    break;
  }
}
#endif //PLUS_OBSERVER

/*********************************************************************
* @fn      JC_BLEPeripheral_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t JC_BLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  
  switch (pMsg->event)
  {
#ifdef PLUS_OBSERVER   
    case GAP_MSG_EVENT:
      // Process GATT message
      JC_BLEPeripheralObserver_processRoleEvent((gapPeripheralObserverRoleEvent_t *)pMsg);
      break;
#endif   
  case GATT_MSG_EVENT:
    // Process GATT message
    safeToDealloc = JC_BLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
    break;
    
  case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
      case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Event
        break;
        
      default:
        break;
      }
    }
    break;
    
  default:
    // do nothing
    break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
* @fn      JC_BLEPeripheral_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t JC_BLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      JC_BLEPeripheral_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
    LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                           10, LCD_PAGE5);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      JC_BLEPeripheral_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void JC_BLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      JC_BLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE5);
    }
  }
}

/*********************************************************************
* @fn      JC_BLEPeripheral_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void JC_BLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE5);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE5);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      JC_BLEPeripheral_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void JC_BLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
  case SBP_STATE_CHANGE_EVT:
    JC_BLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                           hdr.state);
    break;
    
  case SBP_CHAR_CHANGE_EVT:
    JC_BLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
    break;
#ifdef PLUS_OBSERVER
    case SBP_OBSERVER_STATE_CHANGE_EVT:
      JC_BLEPeripheral_processStackMsg((ICall_Hdr *)pMsg->pData);
      
      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;
      
#endif
    default:
      // Do nothing.
      break;
    }
  }
  
#ifdef PLUS_OBSERVER

/*********************************************************************
* @fn      JC_BLEPeripheral_ObserverStateChangeCB
*
* @brief   Peripheral observer event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static void JC_BLEPeripheral_ObserverStateChangeCB(gapPeripheralObserverRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  JC_BLEPeripheral_enqueueMsg(SBP_OBSERVER_STATE_CHANGE_EVT, SUCCESS, (uint8_t *)pEvent);
}

#endif
/*********************************************************************
* @fn      JC_BLEPeripheral_stateChangeCB
*
* @brief   Callback from GAP Role indicating a role state change.
*
* @param   newState - new state
*
* @return  None.
*/
static void JC_BLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  JC_BLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

extern uint8 lora_ieee_addr[6];

/*********************************************************************
* @fn      JC_BLEPeripheral_processStateChangeEvt
*
* @brief   Process a pending GAP Role state change event.
*
* @param   newState - new state
*
* @return  None.
*/
static void JC_BLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
    static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER
    
    switch ( newState )
    {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
#ifdef TI_DRIVERS_LORA_ENABLE
#if defined(LORA_MSPAN_ENABLE) || defined(LORA_JCNETPAN_ENABLE) || defined(LORA_WAN_ENABLE)
        
        // set middle bytes to zero
        
        // shift three bytes up
        
        // LoRaNetwork_SetAddr(ownAddress);
        
#endif
#endif //TI_DRIVERS_LORA_ENABLE
        //DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
#ifdef UART_PRINT_ENABLE
        printS("ownAddress:");
        printS((uint8 *)Util_convertBdAddr2Str(ownAddress));
        printS("Initialized.");
#endif //printf
        // Display device address
        LCD_WRITE_STRING(Util_convertBdAddr2Str(ownAddress), LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
      }
      break;
      
    case GAPROLE_ADVERTISING:
      LCD_WRITE_STRING("Advertising", LCD_PAGE2);
      // Production indicate
      PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);
      
#ifdef UART_PRINT_ENABLE			
      printS("Advertising Start!");
#endif
      break;
      
#ifdef PLUS_BROADCASTER   
      /* After a connection is dropped a device in PLUS_BROADCASTER will continue
      * sending non-connectable advertisements and shall sending this change of
      * state to the application.  These are then disabled here so that sending
      * connectable advertisements can resume.
      */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;
        
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        advertEnabled = TRUE;
        
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        // Reset flag for next connection.
        firstConnFlag = false;
        
        JC_BLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER   
      
    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];
        
        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        
        Util_startClock(&periodicClock);
#ifdef APP_HANDSHAKE_ENABLE
        // Production indicate
        PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);
        app_handshake_received = 0;
        app_handshake_pwd_flag = 0;
        App_handshake_Profile_SetParameter(APP_HANDSHAKE_PROFILE_CHAR2, sizeof(uint8_t), &app_handshake_pwd_flag);
#endif // APP_HANDSHAKE_ENABLE
        
#ifdef UART_PRINT_ENABLE			
        printS("Connected!");
        printS("peerAddress:");
        printS((uint8 *)Util_convertBdAddr2Str(peerAddress));
#endif        
        LCD_WRITE_STRING("Connected", LCD_PAGE2);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);
        
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if (firstConnFlag == false)
        {
          uint8_t advertEnabled = FALSE; // Turn on Advertising
          
          // Disable connectable advertising.
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                               &advertEnabled);
          
          // Set to true for non-connectabel advertising.
          advertEnabled = TRUE;
          
          // Enable non-connectable advertising.
          GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                               &advertEnabled);
          firstConnFlag = true;
        }
#endif // PLUS_BROADCASTER
      }
      break;
      
    case GAPROLE_CONNECTED_ADV:
#ifdef UART_PRINT_ENABLE			
      printS("Connected Advertising...");
#endif  //Printf
      LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;
      
    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      JC_BLEPeripheral_freeAttRsp(bleNotConnected);
#ifdef UART_PRINT_ENABLE			
      printS("Disconnected!");
#endif //Printf
      LCD_WRITE_STRING("Disconnected", LCD_PAGE2);
      
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
      break;
      
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      JC_BLEPeripheral_freeAttRsp(bleNotConnected);
      
      LCD_WRITE_STRING("Timed Out", LCD_PAGE2);
#ifdef UART_PRINT_ENABLE			
      Util_stopClock(&periodicClock);
      printS("Timed Out!");
#endif //Printf
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
      
#ifdef PLUS_BROADCASTER
      // Reset flag for next connection.
      firstConnFlag = false;
#endif //#ifdef (PLUS_BROADCASTER)
      break;
      
    case GAPROLE_ERROR:
#ifdef UART_PRINT_ENABLE			
      printS("Error!");
#endif //Printf
    LCD_WRITE_STRING("Error", LCD_PAGE2);
    break;
    
  default:
    LCD_WRITE_STRING("", LCD_PAGE2);
    break;
  }
  // Update the state
  //gapProfileState = newState;
}

//#ifndef FEATURE_OAD
/*********************************************************************
* @fn      JC_BLEPeripheral_charValueChangeCB
*
* @brief   Callback from JC Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void JC_BLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  JC_BLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID, NULL);
}
// #endif //!FEATURE_OAD

/*********************************************************************
* @fn      JC_BLEPeripheral_processCharValueChangeEvt
*
* @brief   Process a pending JC Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void JC_BLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
  //#ifndef FEATURE_OAD
  uint8_t newValue[20];
    uint8_t app_flag;

    switch(paramID)
    {
    case JC_PROFILE_HANDSHAKE_CHAR:
      App_handshake_Profile_GetParameter(JC_PROFILE_HANDSHAKE_CHAR, newValue);
      app_flag = 0;
      if (app_handshake_received) 
      {
#ifndef USE_MD5
        if (memcmp(&newValue[1], UserCharAppPwd, newValue[0]))
        {
          VOID memcpy(UserCharAppPwd, &newValue[1], newValue[0]);//updata pwd
          VOID osal_snv_write(BLE_NVID_USER_SET_APP_PWD, newValue[0], &newValue[1]);//app pwd
        }
        if (UserCharAppFlag == 0) {
          UserCharAppFlag = 1;
          VOID osal_snv_write(BLE_NVID_USER_SET_APP_FLAG, 1, &UserCharAppFlag);//app set flag
        }
        scanRspData[sizeof(scanRspData) - 3] = UserCharAppFlag;
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                             scanRspData);
        app_flag = 2;//Password is changed
#else
        app_flag = 1;
#endif
      }
      else 
      {
        if (!memcmp(&newValue[1], UserCharAppPwd, newValue[0])) 
        {
          app_handshake_received = 1;
          Util_stopClock(&periodicClock);
          app_flag = 1;//Password authentication through
        }
      }
      App_handshake_Profile_SetParameter(APP_HANDSHAKE_PROFILE_CHAR1, sizeof(uint8_t), &app_flag);
      break;
    case APP_HANDSHAKE_PROFILE_CHAR2:
#ifndef USE_MD5
      if (UserCharAppFlag != 0 && app_handshake_received > 0) {
        uint8 tmp = 0xfe;
        char default_pwd[6] = "123456";
        UserCharAppFlag = 0;
        VOID memcpy(UserCharAppPwd, default_pwd, sizeof(default_pwd));//updata pwd
        VOID osal_snv_write(BLE_NVID_USER_SET_APP_PWD, sizeof(default_pwd), default_pwd);//app pwd
        VOID osal_snv_write(BLE_NVID_USER_SET_APP_FLAG, 1, &UserCharAppFlag);//app set flag
        scanRspData[sizeof(scanRspData) - 3] = UserCharAppFlag;
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),scanRspData);
        App_handshake_Profile_SetParameter(APP_HANDSHAKE_PROFILE_CHAR2, sizeof(uint8_t), &tmp);
      }
#endif
      break;
#ifdef APP_CONTROL_ENABLE
    case APP_CONTROL_PROFILE_CHAR3:
      App_Control_Profile_GetParameter(APP_CONTROL_PROFILE_CHAR3, newValue);
      
//      if (newValue[0] & 0x01) {
//
//      }
      
      if ((newValue[0] & 0x02) != module_Ask_and_Lora_status)
      {
        module_Ask_and_Lora_status = (newValue[0] & 0x02);
        VOID osal_snv_write(BLE_NVID_USER_SET_LORA_AND_ASK, 1, &module_Ask_and_Lora_status);
        Process_Ask_and_Lora_Status(module_Ask_and_Lora_status);
        HAL_SYSTEM_RESET();
      }
      break;
#endif
  default:
    // should not reach here!
    break;
  }
  //#endif //!FEATURE_OAD
}

/*********************************************************************
* @fn      JC_BLEPeripheral_performPeriodicTask
*
* @brief   Perform a periodic application task. This function gets called
*          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
*          the value of the third characteristic in the JCGATTProfile
*          service is retrieved from the profile, and then copied into the
*          value of the the fourth characteristic.
*
* @param   None.
*
* @return  None.
*/
static void JC_BLEPeripheral_performPeriodicTask(void)
{
  //#ifndef FEATURE_OAD
  
  // Call to retrieve the value of the third characteristic in the profile
  // Call to set that value of the fourth characteristic in the profile.
  // Note that if notifications of the fourth characteristic have been
  // enabled by a GATT client device, then a notification will be sent
  // every time this function is called.
#ifdef APP_HANDSHAKE_ENABLE
    app_handshake_received = 0;
    // Production indicate
    PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);
    GAPRole_TerminateConnection();
#endif
}
  
  
#if defined(FEATURE_OAD)
  /*********************************************************************
  * @fn      JC_BLEPeripheral_processOadWriteCB
  *
  * @brief   Process a write request to the OAD profile.
  *
  * @param   event      - event type:
  *                       OAD_WRITE_IDENTIFY_REQ
  *                       OAD_WRITE_BLOCK_REQ
  * @param   connHandle - the connection Handle this request is from.
  * @param   pData      - pointer to data for processing and/or storing.
  *
  * @return  None.
  */
void JC_BLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                          uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
    sizeof(uint8_t) * OAD_PACKET_SIZE);
  
  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;
    
    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);
    
    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);
    
    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
* @fn      JC_BLEPeripheral_clockHandler
*
* @brief   Handler function for clock timeouts.
*
* @param   arg - event type
*
* @return  None.
*/
static void JC_BLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;
  
  // Wake up the application.
  Semaphore_post(sem);
}
/*********************************************************************
* @fn      JC_BLEPeripheral_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   state - message state.
*
* @return  None.
*/
static void JC_BLEPeripheral_enqueueMsg(uint8_t event, uint8_t state, 
                                        uint8_t *pData)
{
  sbpEvt_t *pMsg;
  
  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;
    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}
/*********************************************************************
* @fn      SensorTag_blinkLed
*
* @brief   Blinkk a led 'n' times, duty-cyle 50-50
* @param   led - led identifier
* @param   nBlinks - number of blinks
*
* @return  none
*/
void JC_BLEPeripheral_blinkLed(uint8_t led, uint8_t nBlinks)
{
  uint8_t i;
  
  for (i=0; i<nBlinks; i++)
  {
    PIN_setOutputValue(hGpioPin, led, Board_LED_ON);
    delay_ms(2);
    PIN_setOutputValue(hGpioPin, led, Board_LED_OFF);
    delay_ms(2);
  }
}

void JC_blinkLed(void)
{
  JC_BLEPeripheral_blinkLed(Board_LED2, 1);
}

/*********************************************************************
* @fn      JC_BLEPeripheral_Battery
*
* @brief   JC_BLEPeripheral_Battery
*
* @return  percent.
*/
static uint32_t JC_BLEPeripheral_Battery(void)
{
  uint32_t percent = 0;
#ifndef JCSAVER_INCLUDED
  uint16_t BattMaxLevel = 3000; //3.27V--100%
  uint16_t BattMinLevel = 2000;//2.0V--0%
  
  // Read the battery voltage (V), only the first 12 bits
  
  percent = AONBatMonBatteryVoltageGet();
  //percent = AONBatMonTemperatureGet();
  // Convert to from V to mV to avoid fractions.
  // Fractional part is in the lower 8 bits thus converting is done as follows:
  // (1/256)/(1/1000) = 1000/256 = 125/32
  // This is done most effectively by multiplying by 125 and then shifting
  // 5 bits to the right.
  percent = (percent * 125) >> 5;
  scanRspData[sizeof(scanRspData) - 1] = AONBatMonTemperatureGetDegC();//temp
#ifndef IBEACON_ENABLE
  advertData[sizeof(advertData) - 1] = AONBatMonTemperatureGetDegC();
#endif
  
  if (percent > BattMaxLevel) {
    percent = 100;
  }  else {
    if (percent < BattMinLevel)  percent = BattMinLevel;
    percent = (((percent - BattMinLevel)* 100) / (BattMaxLevel - BattMinLevel));
  }
#endif
  return percent;
  
}

/*********************************************************************
* @fn      JC_BLEPeripheral_BattMeasure
*
* @brief   JC_BLEPeripheral_BattMeasure
*
* @return  none.
*/
static void JC_BLEPeripheral_BattMeasure(void)
{
  uint32_t percent = 0;
  uint16_t BattMaxLevel = 3000;//5000; //5V--100%
  uint16_t BattMinLevel = 1000;//4000;//2.0V--0%
#ifdef UART_PRINT_ENABLE
  uint8_t tmpbuf[24] = {0};
#endif

  percent = JC_BLEPeripheral_Battery();

  scanRspData[sizeof(scanRspData) - 2] = percent;
  
#ifdef UART_PRINT_ENABLE
  memset(tmpbuf,0,20);
  sprintf((char*)tmpbuf,"Battery:%ld",percent);
  printS(tmpbuf);
#endif
}

uint8_t Get_BattMeasure(void)
{
  return scanRspData[sizeof(scanRspData) - 2];
}

#ifdef TI_DRIVERS_LORA_ENABLE 
  static volatile bool PMSetConstraint = FALSE;
#ifdef POWER_SAVING
  // -----------------------------------------------------------------------------
  //! \brief      This routine is used to set constraints on power manager
  //!
  //! \return     void
  // -----------------------------------------------------------------------------
void JC_BLEPeripheral_setPM(void)
{
  if( PMSetConstraint )
  {
    return;
  }
  // set constraints for Standby and idle mode
  Power_setConstraint(Power_SB_DISALLOW);
  Power_setConstraint(Power_IDLE_PD_DISALLOW);
  PMSetConstraint = TRUE;
}
#endif
  
#ifdef POWER_SAVING
  // -----------------------------------------------------------------------------
  //! \brief      This routine is used to release constraints on power manager
  //!
  //! \return     void
  // -----------------------------------------------------------------------------
void JC_BLEPeripheral_relPM(void)
{
  if ( ! PMSetConstraint )
  {
    return;
  }
  // release constraints for Standby and idle mode
  Power_releaseConstraint(Power_SB_DISALLOW);
  Power_releaseConstraint(Power_IDLE_PD_DISALLOW);
  PMSetConstraint = FALSE;
}
#endif
#endif //TI_DRIVERS_LORA_ENABLE
  
  /**@brief   Function for the light initialization.
  *
  * @details Initializes all lights used by this application.
  */
#ifdef USE_MD5
  //#pragma optimize=none
void JC_BLEPeripheral_Get_md5(uint8_t *serial_number, uint8_t len)
{
    char device_name[] = DEVICE_NAME;
    char *argv=NULL;
    
    argv = (char *)malloc(len + sizeof(DEVICE_NAME));
    //serial number and name 
    memcpy(argv, serial_number, len);
    memcpy(argv + len, device_name, strlen(DEVICE_NAME));
    md5(argv, len + strlen(device_name), Md5_serial_id_and_name);
    free(argv);
}
#endif
  
#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
#include "UartDataProcess.h"
  //#pragma optimize=none
static void JC_BLEPeripheral_TLpacketParser(void)
{
  //read available bytes
  uint8_t len = TLgetRxBufLen();
  if (len >= APP_TL_BUFF_SIZE)
  {
    len = APP_TL_BUFF_SIZE;
  }
  TLread(appRxBuf, len);
  if (PTM_Uart_enable) {
    UartDataProcess(appRxBuf, len);
  } else {
    
  }
  // ADD PACKET PARSER HERE
  // for now we just echo...
}

uint8_t *Get_ModuleVersion(void)
{
  return &advertData[sizeof(advertData) - 4];
}

#endif //NPI_USE_SPI

static void Process_Ask_and_Lora_Status(uint8_t stat)
{
#ifdef TI_DRIVERS_LORA_ENABLE
#if defined( LORA_MSPAN_ENABLE ) || defined( LORA_JCNETPAN_ENABLE ) || defined( LORA_WAN_ENABLE )
  if (stat & 0x02) { 
    lora_enable = 1;
  } else {
    lora_enable = 0;
  }
#endif
#endif
}
/*********************************************************************
*********************************************************************/
  