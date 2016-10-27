/**************************************************************************************************
  Filename:       temperature_sensor.h
  date:           add by shenhz at 20161014
  describe:       温湿度传感器

**************************************************************************************************/
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <ti/drivers/PIN.h>	
#include <ti/sysbios/knl/Clock.h>
#include "util.h"
  
typedef struct dht11_data{
  uint8_t     temp_value;
  uint8_t     humi_value;
  
}dht11_data_t;

/*********************************************************************
 * @fn      temperatureSensorInit
 *
 * @brief   init DHT11sensor
 *
 * @param   none
 *
 * @return  none
 */
void temperatureSensorInit(void);
/*********************************************************************
 * @fn      temperatureSensorClose
 *
 * @brief   close DHT11 sensor
 *
 * @param   none
 *
 * @return  none
 */
void temperatureSensorClose(void);
/*********************************************************************
 * @fn      temperatureSensorRead
 *
 * @brief   read DHT11 sensor's value
 *
 * @param   dht11_value
 *
 * @return  0 - ok, 1 - err
 */
int temperatureSensorRead(dht11_data_t* dht11_value);

/*********************************************************************
 * @fn      getTemperature
 *
* @brief   循环采样（2s）
 *
 * @param   dht11_value
 *
 * @return  0 - ok, 1 - err
 */
int getTemperature(dht11_data_t* dht11_value);

/*********************************************************************
 * @fn      getTsensorValue
 *
* @brief   获取温湿度
 *
 * @param   dht11_value
 *
 * @return  0 - ok, 1 - err
 */
int getTsensorValue(dht11_data_t* dht11_value);

#ifdef __cplusplus
}
#endif

#endif /* TEMPERATURE_SENSOR_H */
