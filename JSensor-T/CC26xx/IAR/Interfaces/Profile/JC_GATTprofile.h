/**************************************************************************************************
  Filename:       JC_GATTprofile.h
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the JC_ GATT profile definitions and
                  prototypes.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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

#ifndef JC_GATTPROFILE_H
#define JC_GATTPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define JC_PROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value 
#define JC_PROFILE_CHAR2                   1  // RW uint8 - Profile Characteristic 2 value
#define JC_PROFILE_CHAR3                   2  // RW uint8 - Profile Characteristic 3 value
#define JC_PROFILE_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value
#define JC_PROFILE_CHAR5                   4  // RW uint8 - Profile Characteristic 5 value
#define JC_PROFILE_CHAR6                   5  // RW uint8 - Profile Characteristic 6 value
//#define JC_PROFILE_HANDSHAKE_CHAR          6
  
// JC_ Profile Service UUID
#define JC_PROFILE_SERV_UUID               0xFFF0
    
// Key Pressed UUID
#define JC_PROFILE_CHAR1_UUID             0xFFF1
#define JC_PROFILE_CHAR2_UUID             0xFFF2
#define JC_PROFILE_CHAR3_UUID             0xFFF3
#define JC_PROFILE_CHAR4_UUID             0xFFF4
#define JC_PROFILE_CHAR5_UUID             0xFFF5
#define JC_PROFILE_CHAR6_UUID             0xFFF6
  
// JC_ Keys Profile Services bit fields
#define JC_PROFILE_SERVICE                0x00000001

// Length of Characteristic 5 in bytes
#define JC_PROFILE_CHAR5_LEN              5  
#define VER_LEN			                      2
#define MAJOR_MINOR_LEN		                4
#define ADVERTISING_INTERVAL_LEN          2
#define POWER_TX_LEN                      2
#define ADJUST_LEN                        4
 
#define BLE_NVID_USER_SET_UUID          BLE_NVID_CUST_START  //!< User set UUID NV IDs
#define BLE_NVID_USER_SET_MMID          BLE_NVID_CUST_START+1  //!< User set major and minor NV IDs
#define BLE_NVID_USER_SET_ADV_INT       BLE_NVID_CUST_START+2	//!< User set ADVERTISING INTERVAL NV IDs
#define BLE_NVID_USER_SET_POWER         BLE_NVID_CUST_START+3	//!< User set Power NV IDs
#define BLE_NVID_USER_SET_ADJUST        BLE_NVID_CUST_START+4
#ifdef TI_DRIVERS_LORA_ENABLE
#define BLE_NVID_USER_SET_LORA_SPEED    BLE_NVID_CUST_START+5  //组网速率  
#endif


#define BLE_NVID_USER_SET_APP_PWD       BLE_NVID_CUST_START+6
#define BLE_NVID_USER_SET_APP_FLAG      BLE_NVID_CUST_START+7
#define BLE_NVID_USER_SET_APP_STAT      BLE_NVID_CUST_START+8  //路灯状态
#define BLE_NVID_USER_SET_APP_DIM       BLE_NVID_CUST_START+9  //路灯亮度

  
#ifdef  USE_RFM
#define BLE_NVID_USER_ASK_PSW           BLE_NVID_CUST_START+10
#endif
  
#ifdef USE_MD5
#define BLE_NVID_USER_SERIAL_NUMBER     BLE_NVID_CUST_START+11
#endif

#define BLE_NVID_USER_SET_LORA_AND_ASK  BLE_NVID_CUST_START+12
//#define BLE_NVID_USER_MAG_SYS_INFO      BLE_NVID_CUST_START+13

/********************************************************************
 * TYPEDEFS
 */

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*JC_ProfileChange_t)( uint8 paramID );

typedef struct
{
  JC_ProfileChange_t        pfnJC_ProfileChange;  // Called when characteristic value changes
} JC_ProfileCBs_t;

    

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * JC_Profile_AddService- Initializes the JC_ GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t JC_Profile_AddService( uint32 services );

/*
 * JC_Profile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t JC_Profile_RegisterAppCBs( JC_ProfileCBs_t *appCallbacks );

/*
 * JC_Profile_SetParameter - Set a JC_ GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t JC_Profile_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * JC_Profile_GetParameter - Get a JC_ GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t JC_Profile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* JC_GATTPROFILE_H */
