/**************************************************************************************************
  Filename:       app_control_service.c
  date:           add by shenhz at 20161014
  describe:       app应用ble控制（读/写）统一服务定义接口

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

#include "temperature_sensor.h"//在具体应用中实现具体控制操作
#include "app_control_service.h"
#include "app_handshake_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        10

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// PLC Profile Service UUID: 0xEFF0
CONST uint8_t appControlProfileServUUID[ATT_UUID_SIZE] =
{ 
	TI_BASE_UUID_128(APP_CONTROL_PROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xEFF1
CONST uint8_t appControlProfilechar1UUID[ATT_UUID_SIZE] =
{ 
	TI_BASE_UUID_128(APP_CONTROL_PROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xEFF2
CONST uint8_t appControlProfilechar2UUID[ATT_UUID_SIZE] =
{ 
	TI_BASE_UUID_128(APP_CONTROL_PROFILE_CHAR2_UUID)
};

CONST uint8_t appControlProfilechar3UUID[ATT_UUID_SIZE] =
{ 
	TI_BASE_UUID_128(APP_CONTROL_PROFILE_CHAR3_UUID)
};

CONST uint8_t appControlProfilechar4UUID[ATT_UUID_SIZE] =
{ 
	TI_BASE_UUID_128(APP_CONTROL_PROFILE_CHAR4_UUID)
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

static JC_ProfileCBs_t *appControlProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// PLC Profile Service attribute
static CONST gattAttrType_t appControlProfileService = { ATT_UUID_SIZE, appControlProfileServUUID};


// PLC Profile Characteristic 1 Properties
static uint8_t appControlProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8_t appControlProfileChar1 = 0;//10 byte control sem


// Plc Profile Characteristic 2 Properties
static uint8_t appControlProfileChar2Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic 2 Value
static uint8_t appControlProfileChar2 = 0;// on-off state

// JC_ Profile Characteristic 2 User Description
//static uint8_t JC_ProfileChar2UserDesp[17] = "Characteristic 2";
static gattCharCfg_t *appControlProfileChar2Cfg;

static uint8_t appControlProfileChar3Props = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t appControlProfileChar3 = 0;

static uint8_t appControlProfileChar4Props = GATT_PROP_READ;
static uint8_t appControlProfileChar4 = 0;
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t appControlProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // JC_ Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&appControlProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &appControlProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_UUID_SIZE, appControlProfilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        &appControlProfileChar1 
      },
			
    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &appControlProfileChar2Props 
    },

      // Characteristic Value 2
      { 
        { ATT_UUID_SIZE, appControlProfilechar2UUID },
        GATT_PERMIT_READ, 
        0, 
        &appControlProfileChar2 
      },
		
      // Characteristic 2 configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8_t *)&appControlProfileChar2Cfg 
      },
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &appControlProfileChar3Props 
    },
      { 
        { ATT_UUID_SIZE, appControlProfilechar3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        &appControlProfileChar3 
      },
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &appControlProfileChar4Props 
    },
      { 
        { ATT_UUID_SIZE, appControlProfilechar4UUID },
        GATT_PERMIT_READ,
        0, 
        &appControlProfileChar4 
      },      
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t appControlProfile_ReadAttrCB( uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method );
static bStatus_t appControlProfile_WriteAttrCB( uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Plc Profile Service Callbacks
CONST gattServiceCBs_t appControlProfileCBs =
{
  appControlProfile_ReadAttrCB,  // Read callback function pointer
  appControlProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      App_Control_Profile_AddService
 *
 * @brief   Initializes the Plc Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t App_Control_Profile_AddService( void )
{
  uint8_t status;
  // Allocate Client Characteristic Configuration table
  appControlProfileChar2Cfg = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );
  if ( appControlProfileChar2Cfg == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, appControlProfileChar2Cfg );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( appControlProfileAttrTbl, 
                                          GATT_NUM_ATTRS( appControlProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &appControlProfileCBs );
  return ( status );
}

/*********************************************************************
 * @fn      App_Control_Profile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t App_Control_Profile_RegisterAppCBs( JC_ProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    appControlProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      App_Control_Profile_SetParameter
 *
 * @brief   Set a Plc Profile parameter.
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
bStatus_t App_Control_Profile_SetParameter( uint8_t param, uint8_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case APP_CONTROL_PROFILE_CHAR1:
      if ( len == sizeof(uint8_t) ) //1 byte on 0ff sem
      {
				appControlProfileChar1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
			
		case APP_CONTROL_PROFILE_CHAR2:
      if ( len == sizeof(uint8_t) ) 
      {
	appControlProfileChar2 = *((uint8*)value);
        //memcpy(appControlProfileChar2, (uint8*)value, 2);
	ret = GATTServApp_ProcessCharCfg( appControlProfileChar2Cfg, &appControlProfileChar2, FALSE,
                                    appControlProfileAttrTbl, GATT_NUM_ATTRS( appControlProfileAttrTbl ),
                                    INVALID_TASK_ID, appControlProfile_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
  case APP_CONTROL_PROFILE_CHAR4:
    if ( len == sizeof(uint8_t) )
    {
      appControlProfileChar4 = *((uint8*)value);
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
 * @fn      App_Control_Profile_GetParameter
 *
 * @brief   Get a Plc Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t App_Control_Profile_GetParameter( uint8_t param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case APP_CONTROL_PROFILE_CHAR1:
      VOID memcpy( value, &appControlProfileChar1, sizeof(uint8_t) );
      break;

    case APP_CONTROL_PROFILE_CHAR2:
      VOID memcpy( value, &appControlProfileChar2, sizeof(uint8_t) );
      break;      
    case APP_CONTROL_PROFILE_CHAR3:
      VOID memcpy( value, &appControlProfileChar3, sizeof(uint8_t) );
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          appControlProfile_ReadAttrCB
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
static bStatus_t appControlProfile_ReadAttrCB( uint16_t connHandle,
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
     if (!memcmp(pAttr->type.uuid, appControlProfilechar1UUID, ATT_UUID_SIZE))
     {
	*pLen = 1;
  	VOID memcpy( pValue, pAttr->pValue, 1 );
     } 
     else if (!memcmp(pAttr->type.uuid, appControlProfilechar2UUID, ATT_UUID_SIZE))
     {
	*pLen = 1;
        pValue[0] = *pAttr->pValue;
  	//VOID memcpy( pValue, pAttr->pValue, 1 );
     }
     else if (!memcmp(pAttr->type.uuid, appControlProfilechar3UUID, ATT_UUID_SIZE))
     {
	*pLen = 1;
        pValue[0] = module_Ask_and_Lora_status;
     }
     else if (!memcmp(pAttr->type.uuid, appControlProfilechar4UUID, ATT_UUID_SIZE))
     {
	*pLen = 1;
        pValue[0] = *pAttr->pValue;
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
 * @fn     appControlProfile_WriteAttrCB
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
static bStatus_t appControlProfile_WriteAttrCB( uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
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
     if (!memcmp(pAttr->type.uuid, appControlProfilechar1UUID, ATT_UUID_SIZE))
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
              uint8_t *pCurValue = (uint8_t *)pAttr->pValue;  
              VOID memcpy( pCurValue, pValue, len );
              //notifyApp = APP_CONTROL_PROFILE_CHAR1;
              if (app_handshake_received)
             {
                if (*pCurValue == 0x07) {
                //PLC_Lock();
                //App_Control_Opera_Service(1);
             } else if (*pCurValue == 0x08) {
               //PLC_Unlock();
               //App_Control_Opera_Service(1);
             } 
          }
       }
    } 
    else if (!memcmp(pAttr->type.uuid, appControlProfilechar3UUID, ATT_UUID_SIZE))
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
           uint8 *pCurValue = (uint8 *)pAttr->pValue;        
           *pCurValue = pValue[0];
           notifyApp = APP_CONTROL_PROFILE_CHAR3;
       }
    }
    else 
    {
       status = ATT_ERR_ATTR_NOT_FOUND;
		}
       //status = ATT_ERR_INVALID_HANDLE;
  }
  if ( (notifyApp != 0xFF ) && appControlProfile_AppCBs && appControlProfile_AppCBs->pfnJC_ProfileChange )
  {
    appControlProfile_AppCBs->pfnJC_ProfileChange( notifyApp );  
  }
  return ( status );
}

/*********************************************************************
*********************************************************************/
