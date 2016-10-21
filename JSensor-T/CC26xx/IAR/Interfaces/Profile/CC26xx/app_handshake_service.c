/**************************************************************************************************
  Filename:       App_handshake_service.c
  Revised:        $Date: 2014-12-23 09:30:19 -0800 (Tue, 23 Dec 2014) $
  Revision:       $Revision: 41572 $

  Description:    This file contains the JC_ GATT profile sample GATT service 
                  profile for use with the BLE sample application.

  Copyright 2010 - 2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "util.h"
#include "JC_GATTprofile.h"
#include "app_handshake_service.h"

#include <ti/drivers/PIN.h>	
#include "Board.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// App_handshake Profile Service UUID: 0xEFF0
CONST uint8_t App_handshake_ProfileServUUID[ATT_UUID_SIZE] =
{ 
	TI_BASE_UUID_128(APP_HANDSHAKE_PROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xDFF1
CONST uint8_t App_handshake_Profilechar1UUID[ATT_UUID_SIZE] =
{
	TI_BASE_UUID_128(APP_HANDSHAKE_PROFILE_CHAR1_UUID)
};
// Characteristic 2 UUID: 0xDFF2
CONST uint8_t App_handshake_Profilechar2UUID[ATT_UUID_SIZE] =
{
	TI_BASE_UUID_128(APP_HANDSHAKE_PROFILE_CHAR2_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern PIN_Handle hGpioPin;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static JC_ProfileCBs_t *App_handshake_Profile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// App_handshake Profile Service attribute
static CONST gattAttrType_t App_handshake_ProfileService = { ATT_UUID_SIZE, App_handshake_ProfileServUUID};

// App_handshake Profile Characteristic 1 Properties
static uint8_t App_handshake_ProfileChar1Props = GATT_PROP_WRITE | GATT_PROP_READ;

static uint8_t App_handshake_ProfileChar2Props = GATT_PROP_WRITE | GATT_PROP_READ;
static uint8_t App_handshake_ProfileChar1[20] = {0};//6 byte
static uint8_t App_handshake_ProfileChar2 = 0;//1 byte
static uint8_t app_handshake_clear_flag;

uint8_t app_handshake_received;
uint8_t app_handshake_pwd_flag;
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t App_handshake_ProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // JC_ Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&App_handshake_ProfileService  /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &App_handshake_ProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_UUID_SIZE, App_handshake_Profilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        App_handshake_ProfileChar1
      },  
       // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &App_handshake_ProfileChar2Props 
    },
      // Characteristic Value 2
      { 
        { ATT_UUID_SIZE, App_handshake_Profilechar2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        &App_handshake_ProfileChar2
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t App_handshake_Profile_ReadAttrCB( uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method );
static bStatus_t App_handshake_Profile_WriteAttrCB( uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);
/*********************************************************************
 * PROFILE CALLBACKS
 */
// App_handshake Profile Service Callbacks
CONST gattServiceCBs_t App_handshake_ProfileCBs =
{
  App_handshake_Profile_ReadAttrCB,  // Read callback function pointer
  App_handshake_Profile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      JC_App_handshake_Profile_AddService
 *
 * @brief   Initializes the App_handshake Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t JC_App_handshake_Profile_AddService( void )
{
  uint8_t status;
  app_handshake_received = 0;
  app_handshake_clear_flag  = 0;

#if 0
  // Allocate Client Characteristic Configuration table
  App_handshake_ProfileChar2Cfg = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );
  if ( App_handshake_ProfileChar2Cfg == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, plcProfileChar2Cfg );
#endif
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( App_handshake_ProfileAttrTbl, 
                                          GATT_NUM_ATTRS( App_handshake_ProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &App_handshake_ProfileCBs );
  return ( status );
}

/*********************************************************************
 * @fn      App_handshake_Profile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t App_handshake_Profile_RegisterAppCBs( JC_ProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    App_handshake_Profile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      App_handshake_Profile_SetParameter
 *
 * @brief   Set a App_handshake Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t App_handshake_Profile_SetParameter( uint8_t param, uint8_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case APP_HANDSHAKE_PROFILE_CHAR1:
      if ( len == sizeof(uint8_t) ) //1 byte on 0ff sem
      {
        app_handshake_pwd_flag = *((uint8*)value);
				//App_handshake_ProfileChar1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case APP_HANDSHAKE_PROFILE_CHAR2:
      if ( len == sizeof(uint8_t) ) //1 byte on 0ff sem
      {
        app_handshake_clear_flag = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      App_handshake_Profile_GetParameter
 *
 * @brief   Get a App_handshake Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t App_handshake_Profile_GetParameter( uint8_t param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case JC_PROFILE_HANDSHAKE_CHAR:
      VOID memcpy( value, App_handshake_ProfileChar1, App_handshake_ProfileChar1[0]+1 );
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          App_handshake_Profile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t App_handshake_Profile_ReadAttrCB( uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method)
{
  bStatus_t status = SUCCESS;
  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_UUID_SIZE )
  {// 128-bit UUID
		if (!memcmp(pAttr->type.uuid, App_handshake_Profilechar1UUID, ATT_UUID_SIZE))
		{
			*pLen = 1;
      pValue[0] = app_handshake_pwd_flag;
      if (app_handshake_pwd_flag == 2) {
        app_handshake_pwd_flag = 1;
      }
  		//VOID memcpy( pValue, pAttr->pValue, 1 );
		} 
    else
    if (!memcmp(pAttr->type.uuid, App_handshake_Profilechar2UUID, ATT_UUID_SIZE))
		{
			*pLen = 1;
      pValue[0] = app_handshake_clear_flag;
      if (app_handshake_clear_flag == 0xFE) {
        app_handshake_clear_flag = 0;
      }
  		//VOID memcpy( pValue, pAttr->pValue, 1 );
		}
		else 
		{
			*pLen = 0;
     	status = ATT_ERR_ATTR_NOT_FOUND;
		}
    
    //*pLen = 0;
    //status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn     App_handshake_Profile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t App_handshake_Profile_WriteAttrCB( uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  uint8 Tmp_len = 0;
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {    
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {// 128-bit UUID
		if (!memcmp(pAttr->type.uuid, App_handshake_Profilechar1UUID, ATT_UUID_SIZE))
		{
      if ( offset == 0 )
			{
        Tmp_len = pValue[0]+1;
        if ( len != Tmp_len )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
			}
			else
			{
        status = ATT_ERR_ATTR_NOT_LONG;
			}
      if ( status == SUCCESS )
      {
        if ((Tmp_len > 6) && (Tmp_len < 18)) {
          uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          notifyApp = JC_PROFILE_HANDSHAKE_CHAR;
          VOID memcpy( pCurValue, pValue, Tmp_len );
        }
      }
		}
    else
    if (!memcmp(pAttr->type.uuid, App_handshake_Profilechar2UUID, ATT_UUID_SIZE))
		{
      if ( offset == 0 )
			{
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
			}
			else
			{
        status = ATT_ERR_ATTR_NOT_LONG;
			}
      if ( status == SUCCESS )
      {
          //uint8 *pCurValue = (uint8 *)pAttr->pValue; 
          if(pValue[0] == 0xFF)
          {
            notifyApp = APP_HANDSHAKE_PROFILE_CHAR2;
            //VOID memcpy( pCurValue, pValue, 1 );
          }
      }
    }
		else 
		{
     	status = ATT_ERR_ATTR_NOT_FOUND;
		}
    //status = ATT_ERR_INVALID_HANDLE;
  }
  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && App_handshake_Profile_AppCBs && App_handshake_Profile_AppCBs->pfnJC_ProfileChange )
  {
    App_handshake_Profile_AppCBs->pfnJC_ProfileChange( notifyApp );  
  }
  return ( status );
}
/*********************************************************************
*********************************************************************/
