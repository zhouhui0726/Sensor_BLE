/**************************************************************************************************
  Filename:      JC_BLEperipheral.h
  Revised:        $Date: 2014-04-16 15:24:18 -0700 (Wed, 16 Apr 2014) $
  Revision:       $Revision: 38209 $

  Description:    This file contains the JC BLE Peripheral sample application
                  definitions and prototypes.

  Copyright 2013 - 2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS�WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef JC_BLEPERIPHERAL_H
#define JC_BLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif
  
 /*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */ 
#ifdef TI_DRIVERS_I2C_INCLUDED
#include "board.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
#define MS_2_TICKS(ms) ( ((ms) * 1000) / Clock_tickPeriod )
#else
#define delay_ms(i) ( CPUdelay(8000*(i)) )
#endif//TI_DRIVERS_I2C_INCLUDED
  
#ifdef TI_DRIVERS_LORA_ENABLE
#include "Board.h"
extern PIN_State pinGpioState;
extern void JC_BLEPeripheral_blinkLed(uint8_t led, uint8_t nBlinks);
#endif

#include <driverlib/aon_batmon.h>


/*********************************************************************
* CONSTANTS
*/
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     40//50ms

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     50//100ms
#else
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         3

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

#ifdef PLUS_OBSERVER
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  10

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 5000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

#endif //#ifdef PLUS_OBSERVER

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   1000
#endif


// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004


#ifdef PLUS_OBSERVER
#define SBP_OBSERVER_STATE_CHANGE_EVT         0x0020  
#endif //PLUS_OBSERVER

#define SBP_CONN_EVT_END_EVT                  0x0008

#define SBP_BATT_CAP_EVT                      0x0080

#define SEND_STAT_TO_BLE_EVT                  0x0100

#define UPDATE_SENSOR_DATA_EVT                0x0200





#define SCAN_RSP_SERV_UUID                    0xF004


/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the JC BLE Peripheral.
 */
extern void JC_BLEPeripheral_createTask(void);

extern void PWM_Run(void);
/*********************************************************************
*********************************************************************/
extern uint8_t Get_BattMeasure(void);

#ifdef __cplusplus
}
#endif

#endif /* JC_BLEPERIPHERAL_H */
