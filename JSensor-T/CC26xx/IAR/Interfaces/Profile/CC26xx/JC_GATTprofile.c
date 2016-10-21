/**************************************************************************************************
Filename:       JC_GATTprofile.c
Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
Revision:       $Revision: 44370 $

Description:    This file contains the JC_ GATT profile sample GATT service 
profile for use with the BLE sample application.

Copyright 2010 - 2015 Texas Instruments Incorporated. All rights reserved.

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
PROVIDED ?S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#include "JC_GATTprofile.h"
#include "hal_mcu.h"
#include "osal_snv.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

#define SERVAPP_NUM_ATTR_SUPPORTED        13//17

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
// JC_ GATT Profile Service UUID: 0xFFF0
CONST uint8 JC_ProfileServUUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(JC_PROFILE_SERV_UUID)
    //LO_UINT16(JC_PROFILE_SERV_UUID), HI_UINT16(JC_PROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 JC_Profilechar1UUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(JC_PROFILE_CHAR1_UUID)
    //LO_UINT16(JC_PROFILE_CHAR1_UUID), HI_UINT16(JC_PROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 JC_Profilechar2UUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(JC_PROFILE_CHAR2_UUID)
    //LO_UINT16(JC_PROFILE_CHAR2_UUID), HI_UINT16(JC_PROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 JC_Profilechar3UUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(JC_PROFILE_CHAR3_UUID)
    //LO_UINT16(JC_PROFILE_CHAR3_UUID), HI_UINT16(JC_PROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 JC_Profilechar4UUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(JC_PROFILE_CHAR4_UUID)
    //LO_UINT16(JC_PROFILE_CHAR4_UUID), HI_UINT16(JC_PROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8 JC_Profilechar5UUID[ATT_UUID_SIZE] =
{ 
  TI_BASE_UUID_128(JC_PROFILE_CHAR5_UUID)
    //LO_UINT16(JC_PROFILE_CHAR5_UUID), HI_UINT16(JC_PROFILE_CHAR5_UUID)
};
// Characteristic 5 UUID: 0xFFF6
CONST uint8 JC_Profilechar6UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(JC_PROFILE_CHAR6_UUID)
    //LO_UINT16(JC_PROFILE_CHAR6_UUID), HI_UINT16(JC_PROFILE_CHAR6_UUID)
};

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/

static JC_ProfileCBs_t *JC_Profile_AppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// JC_ Profile Service attribute
static CONST gattAttrType_t JC_ProfileService = { ATT_UUID_SIZE, JC_ProfileServUUID};


// JC_ Profile Characteristic 1 Properties
static uint8 JC_ProfileChar1Props = GATT_PROP_READ;

// Characteristic 1 Value
static uint8 JC_ProfileChar1[VER_LEN] = {0, 0};//2 byte version

// JC_ Profile Characteristic 1 User Description
//static uint8 JC_ProfileChar1UserDesp[17] = "Characteristic 1";


// JC_ Profile Characteristic 2 Properties
static uint8 JC_ProfileChar2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 2 Value
static uint8 JC_ProfileChar2[ATT_UUID_SIZE] = {0};//128-bit UUID

// JC_ Profile Characteristic 2 User Description
//static uint8 JC_ProfileChar2UserDesp[17] = "Characteristic 2";


// JC_ Profile Characteristic 3 Properties
static uint8 JC_ProfileChar3Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 JC_ProfileChar3[MAJOR_MINOR_LEN] = {0};//major and minor

// JC_ Profile Characteristic 3 User Description
//static uint8 JC_ProfileChar3UserDesp[17] = "Characteristic 3";


// JC_ Profile Characteristic 4 Properties
static uint8 JC_ProfileChar4Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 4 Value
static uint8 JC_ProfileChar4[ADVERTISING_INTERVAL_LEN] = {0};

// JC_ Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
//static gattCharCfg_t *JC_ProfileChar4Config;

// JC_ Profile Characteristic 4 User Description
//static uint8 JC_ProfileChar4UserDesp[17] = "Characteristic 4";


// JC_ Profile Characteristic 5 Properties
static uint8 JC_ProfileChar5Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 5 Value
static uint8 JC_ProfileChar5[POWER_TX_LEN] = {0, 0};
// JC_ Profile Characteristic 5 User Description
//static uint8 JC_ProfileChar5UserDesp[17] = "Characteristic 5";

static uint8 JC_ProfileChar6Props = GATT_PROP_WRITE;

// Characteristic 6 Value
static uint8 JC_ProfileChar6 = 0;




/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t JC_ProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // JC_ Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&JC_ProfileService            /* pValue */
  },
  
  // Characteristic 1 Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &JC_ProfileChar1Props 
  },
  
  // Characteristic Value 1
  { 
    { ATT_UUID_SIZE, JC_Profilechar1UUID },
    GATT_PERMIT_READ, 
    0, 
    JC_ProfileChar1 
  },
#if 0
  // Characteristic 1 User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    JC_ProfileChar1UserDesp 
  },      
#endif
  // Characteristic 2 Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &JC_ProfileChar2Props 
  },
  
  // Characteristic Value 2
  { 
    { ATT_UUID_SIZE, JC_Profilechar2UUID },
    GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_AUTHEN_WRITE, 
    0, 
    JC_ProfileChar2 
  },
#if 0
  // Characteristic 2 User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    JC_ProfileChar2UserDesp 
  },           
#endif
  // Characteristic 3 Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &JC_ProfileChar3Props 
  },
  
  // Characteristic Value 3
  { 
    { ATT_UUID_SIZE, JC_Profilechar3UUID },
    GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_AUTHEN_WRITE, 
    0, 
    JC_ProfileChar3 
  },
#if 0
  // Characteristic 3 User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    JC_ProfileChar3UserDesp 
  },
#endif
  // Characteristic 4 Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &JC_ProfileChar4Props 
  },
  
  // Characteristic Value 4
  { 
    { ATT_UUID_SIZE, JC_Profilechar4UUID },
    GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_AUTHEN_WRITE,
    0, 
    JC_ProfileChar4 
  },
#if 0
  // Characteristic 4 configuration
  { 
    { ATT_BT_UUID_SIZE, clientCharCfgUUID },
    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
    0, 
    (uint8 *)&JC_ProfileChar4Config 
  },
  
  // Characteristic 4 User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    JC_ProfileChar4UserDesp 
  },
#endif
  
  // Characteristic 5 Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &JC_ProfileChar5Props 
  },
  
  // Characteristic Value 5
  { 
    { ATT_UUID_SIZE, JC_Profilechar5UUID },
    GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_AUTHEN_WRITE, 
    0, 
    JC_ProfileChar5 
  },
  #if 0
  // Characteristic 5 User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    JC_ProfileChar5UserDesp 
  },
#endif
  // Characteristic 6 Declaration
  {
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,
    0,
    &JC_ProfileChar6Props
  },
  // Characteristic Value 6
  {
    { ATT_UUID_SIZE, JC_Profilechar6UUID },
    GATT_PERMIT_AUTHEN_WRITE, 
    0,
    &JC_ProfileChar6
  },
};

/*********************************************************************
* LOCAL FUNCTIONS
*/
static bStatus_t JC_Profile_ReadAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr, 
                                       uint8_t *pValue, uint16_t *pLen,
                                       uint16_t offset, uint16_t maxLen,
                                       uint8_t method);
static bStatus_t JC_Profile_WriteAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len,
                                        uint16_t offset, uint8_t method);

/*********************************************************************
* PROFILE CALLBACKS
*/
// JC_ Profile Service Callbacks
CONST gattServiceCBs_t JC_ProfileCBs =
{
  JC_Profile_ReadAttrCB,  // Read callback function pointer
  JC_Profile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      JC_Profile_AddService
*
* @brief   Initializes the JC_ Profile service by registering
*          GATT attributes with the GATT server.
*
* @param   services - services to add. This is a bit map and can
*                     contain more than one service.
*
* @return  Success or Failure
*/
bStatus_t JC_Profile_AddService( uint32 services )
{
  uint8 status;
#if 0
  // Allocate Client Characteristic Configuration table
  JC_ProfileChar4Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                        linkDBNumConns );
  if ( JC_ProfileChar4Config == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, JC_ProfileChar4Config );
#endif
  if ( services & JC_PROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( JC_ProfileAttrTbl, 
                                         GATT_NUM_ATTRS( JC_ProfileAttrTbl ),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &JC_ProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
* @fn      JC_Profile_RegisterAppCBs
*
* @brief   Registers the application callback function. Only call 
*          this function once.
*
* @param   callbacks - pointer to application callbacks.
*
* @return  SUCCESS or bleAlreadyInRequestedMode
*/
bStatus_t JC_Profile_RegisterAppCBs( JC_ProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    JC_Profile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
* @fn      JC_Profile_SetParameter
*
* @brief   Set a JC_ Profile parameter.
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
bStatus_t JC_Profile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case JC_PROFILE_CHAR1:
    if ( len == VER_LEN ) //2 byte version
    {
      VOID memcpy( JC_ProfileChar1, value, VER_LEN );
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case JC_PROFILE_CHAR2:
    if ( len == ATT_UUID_SIZE ) 
    {
      VOID memcpy( JC_ProfileChar2, value, ATT_UUID_SIZE );
      //JC_ProfileChar2 = *((uint8*)value);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case JC_PROFILE_CHAR3:
    if ( len == MAJOR_MINOR_LEN ) 
    {
      VOID memcpy( JC_ProfileChar3, value, MAJOR_MINOR_LEN );
      //JC_ProfileChar3 = *((uint8*)value);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case JC_PROFILE_CHAR4:
    if ( len == ADVERTISING_INTERVAL_LEN ) 
    {
      //JC_ProfileChar4 = *((uint8*)value);
      VOID memcpy( JC_ProfileChar4, value, ADVERTISING_INTERVAL_LEN );
      // See if Notification has been enabled
      //GATTServApp_ProcessCharCfg( JC_ProfileChar4Config, &JC_ProfileChar4, FALSE,
      //JC_ProfileAttrTbl, GATT_NUM_ATTRS( JC_ProfileAttrTbl ),
      //INVALID_TASK_ID, JC_Profile_ReadAttrCB );
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
    
  case JC_PROFILE_CHAR5:
    if ( len == POWER_TX_LEN ) 
    {
      VOID memcpy( JC_ProfileChar5, value, POWER_TX_LEN );
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
* @fn      JC_Profile_GetParameter
*
* @brief   Get a JC_ Profile parameter.
*
* @param   param - Profile parameter ID
* @param   value - pointer to data to put.  This is dependent on
*          the parameter ID and WILL be cast to the appropriate 
*          data type (example: data type of uint16 will be cast to 
*          uint16 pointer).
*
* @return  bStatus_t
*/
bStatus_t JC_Profile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case JC_PROFILE_CHAR1:
    VOID memcpy( value, JC_ProfileChar1, VER_LEN );
    break;
    
  case JC_PROFILE_CHAR2:
    VOID memcpy( value, JC_ProfileChar2, ATT_UUID_SIZE );
    break;      
    
  case JC_PROFILE_CHAR3:
    VOID memcpy( value, JC_ProfileChar3, MAJOR_MINOR_LEN );
    break;  
    
  case JC_PROFILE_CHAR4:
    VOID memcpy( value, JC_ProfileChar4, ADVERTISING_INTERVAL_LEN );
    break;
    
  case JC_PROFILE_CHAR5:
    VOID memcpy( value, JC_ProfileChar5, POWER_TX_LEN );
    break;      
    
  default:
    ret = INVALIDPARAMETER;
    break;
  }
  
  return ( ret );
}

/*********************************************************************
* @fn          JC_Profile_ReadAttrCB
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
static bStatus_t JC_Profile_ReadAttrCB(uint16_t connHandle,
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
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads
      
      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
    case JC_PROFILE_CHAR1_UUID:
    case JC_PROFILE_CHAR2_UUID:
    case JC_PROFILE_CHAR4_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;
      
    case JC_PROFILE_CHAR5_UUID:
      *pLen = JC_PROFILE_CHAR5_LEN;
      VOID memcpy( pValue, pAttr->pValue, JC_PROFILE_CHAR5_LEN );
      break;
      
    default:
      // Should never get here! (characteristics 3 and 4 do not have read permissions)
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }
  }
  else
  {// 128-bit UUID
    if (!memcmp(pAttr->type.uuid, JC_Profilechar1UUID, ATT_UUID_SIZE))
    {
      *pLen = VER_LEN;
      VOID memcpy( pValue, pAttr->pValue, VER_LEN );
    } 
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar2UUID, ATT_UUID_SIZE))
    {
      *pLen = ATT_UUID_SIZE;
      VOID memcpy( pValue, pAttr->pValue, ATT_UUID_SIZE );
    }
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar3UUID, ATT_UUID_SIZE))
    {
      *pLen = MAJOR_MINOR_LEN;
      VOID memcpy( pValue, pAttr->pValue, MAJOR_MINOR_LEN );
    }
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar4UUID, ATT_UUID_SIZE))
    {
      *pLen = ADVERTISING_INTERVAL_LEN;
      VOID memcpy( pValue, pAttr->pValue, ADVERTISING_INTERVAL_LEN );
    }
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar5UUID, ATT_UUID_SIZE))
    {
      *pLen = POWER_TX_LEN;
      VOID memcpy( pValue, pAttr->pValue, POWER_TX_LEN );
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
* @fn      JC_Profile_WriteAttrCB
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
static bStatus_t JC_Profile_WriteAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len,
                                        uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ))
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
    case JC_PROFILE_CHAR1_UUID:
    case JC_PROFILE_CHAR3_UUID:
      
      //Validate the value
      // Make sure it's not a blob oper
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
      
      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        *pCurValue = pValue[0];
        
        if( pAttr->pValue == JC_ProfileChar1 )
        {
          notifyApp = JC_PROFILE_CHAR1;        
        }
        else
        {
          notifyApp = JC_PROFILE_CHAR3;           
        }
      }
      
      break;
      
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
    if (!memcmp(pAttr->type.uuid, JC_Profilechar1UUID, ATT_UUID_SIZE))//Version
    {
      if ( offset == 0 )
      {
        if ( len != VER_LEN )
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
        uint8 *pCurValue = (uint8 *)pAttr->pValue;  
        VOID memcpy( pCurValue, pValue, VER_LEN );
        notifyApp = JC_PROFILE_CHAR1;        
      }
    } 
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar2UUID, ATT_UUID_SIZE))//UUID
    {
      if ( offset == 0 )
      {
        if ( len != ATT_UUID_SIZE )
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
        uint8 *pCurValue = (uint8 *)pAttr->pValue;  
        VOID memcpy( pCurValue, pValue, ATT_UUID_SIZE );
        //printV(pAttr->pValue, len);
        notifyApp = JC_PROFILE_CHAR2;        
      }
    }
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar3UUID, ATT_UUID_SIZE))//major and minor
    {
      if ( offset == 0 )
      {
        if ( len != MAJOR_MINOR_LEN )
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
        uint8 *pCurValue = (uint8 *)pAttr->pValue;  
        VOID memcpy( pCurValue, pValue, MAJOR_MINOR_LEN );
        notifyApp = JC_PROFILE_CHAR3;        
      }
    }
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar4UUID, ATT_UUID_SIZE))//advertising interval
    {
      if ( offset == 0 )
      {
        if ( len != ADVERTISING_INTERVAL_LEN )
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
        
        uint8 *pCurValue = (uint8 *)pAttr->pValue;  
        if (BUILD_UINT16(pValue[1], pValue[0]) <= 20000 && BUILD_UINT16(pValue[1], pValue[0]) >= 20) {
          VOID memcpy( pCurValue, pValue, ADVERTISING_INTERVAL_LEN );
          notifyApp = JC_PROFILE_CHAR4;    
        }
      }
    }
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar5UUID, ATT_UUID_SIZE))
    {
      if ( offset == 0 )
      {
        if ( len != POWER_TX_LEN )
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
        uint8 *pCurValue = (uint8 *)pAttr->pValue;  
        if (pValue[0] <= 12) {
          VOID memcpy( pCurValue, pValue, POWER_TX_LEN );
          notifyApp = JC_PROFILE_CHAR5; 
        }
      }
    }
    else if (!memcmp(pAttr->type.uuid, JC_Profilechar6UUID, ATT_UUID_SIZE))
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
        if (pValue[0] == 1) {
          notifyApp = JC_PROFILE_CHAR6;
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
  if ( (notifyApp != 0xFF ) && JC_Profile_AppCBs && JC_Profile_AppCBs->pfnJC_ProfileChange )
  {
    JC_Profile_AppCBs->pfnJC_ProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/
