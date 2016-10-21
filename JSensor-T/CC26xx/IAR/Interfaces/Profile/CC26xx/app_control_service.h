/**************************************************************************************************
  Filename:       app_control_service.h
  date:           add by shenhz at 20161014
  describe:       app应用ble控制（读/写）统一服务定义接口

**************************************************************************************************/

#ifndef APP_CONTROL_SERVICE_H
#define APP_CONTROL_SERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif
/*********************************************************************
 * INCLUDES
 */
#include "JC_GATTprofile.h"
/*********************************************************************
 * CONSTANTS
 */
// Profile Parameters
#define APP_CONTROL_PROFILE_CHAR1                    1  // RW uint8 - Profile Characteristic 1 value 
#define APP_CONTROL_PROFILE_CHAR2                    2  // RW uint8 - Profile Characteristic 2 value
#define APP_CONTROL_PROFILE_CHAR3                    8  // RW uint8 - Profile Characteristic 3 value 
#define APP_CONTROL_PROFILE_CHAR4                    3  // RW uint8 - Profile Characteristic 4 value 
// JC_ Profile Service UUID 
#define APP_CONTROL_PROFILE_SERV_UUID                0xEFF0

// Key Pressed UUID
#define APP_CONTROL_PROFILE_CHAR1_UUID               0xEFF1 //rev control command
#define APP_CONTROL_PROFILE_CHAR2_UUID               0xEFF2 //notify plc stat

#define APP_CONTROL_PROFILE_CHAR3_UUID               0xEFF3 //on/off lora

#define APP_CONTROL_PROFILE_CHAR4_UUID               0xEFF4 //battery info

extern uint8_t module_Ask_and_Lora_status;

/*********************************************************************
 * TYPEDEFS
 */

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*appControlProfileChange_t)( uint8 paramID );


/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * plcProfile_AddService- Initializes the JC_ GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t App_Control_Profile_AddService( void );

/*
 * App_Control_Profile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t App_Control_Profile_RegisterAppCBs( JC_ProfileCBs_t *appCallbacks );

/*
 * App_Control_Profile_SetParameter - Set a JC_ GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t App_Control_Profile_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * App_Control_Profile_GetParameter - Get a JC_ GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t App_Control_Profile_GetParameter( uint8 param, void *value );

extern void App_Control_Opera_Service(int opera);//在具体应用中实现

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_SERVICE_H */
