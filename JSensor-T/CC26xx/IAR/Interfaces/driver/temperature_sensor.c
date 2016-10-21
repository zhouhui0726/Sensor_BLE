/**************************************************************************************************
  Filename:       temperature_sensor.c
  date:           add by shenhz at 20161014
  describe:       ��ʪ�ȴ�����

**************************************************************************************************/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Board.h"
#include "JC_BLEPeripheral.h"
#include <ti/drivers/pin/PINCC26XX.h>

#ifdef UART_PRINT_ENABLE
#include "uart_printf.h"
#endif

#include "temperature_sensor.h"


#define DHT11_PIN           IOID_18

PIN_Config dht11PinsInCfg[] =
{
  DHT11_PIN      | PIN_INPUT_EN | PIN_PULLUP,
  PIN_TERMINATE
};
PIN_Config dht11PinsOutCfg[] =
{
  DHT11_PIN      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,// | PIN_DRVSTR_MAX,   
  PIN_TERMINATE
};

PIN_State  dht11Pins_s;
PIN_Handle dht11Pins_h;

#define HIGH                        1
#define LOW                         0

//#define USE_FIRST_METHOD    1                                                 //���������ַ���ż����ȷ����������ᶪ����bit����

uint8_t dbgc[90],no=0;

void DHT11_PIN_SetOUT(uint8_t value);
uint8_t DHT11_PIN_GetIN();
void DHT11_Rst(void);
uint8_t DHT11_Check(void);
void delay_us(uint8_t us);
void delay_10us(uint8_t us);

/*********************************************************************
 * @fn      delay_us
 *
 * @brief   delay_us
 *
 * @param   us
 *
 * @return  none
 */
#pragma optimize=none
void delay_us(uint8_t us)
{
  int j=0;
  while(us--) {
    for(j = 0; j < 4; j++);
  }
  
}

/*********************************************************************
 * @fn      delay_10us
 *
 * @brief   delay_us
 *
 * @param   us
 *
 * @return  none
 */
#pragma optimize=none
void delay_10us(uint8_t us)
{
  int j=0;
  while(us--) {
    for(j = 0; j < 50; j++);
  }
  
}

/*********************************************************************
 * @fn      temperatureSensorInit
 *
 * @brief   init DHT11sensor
 *
 * @param   none
 *
 * @return  none
 */
void temperatureSensorInit(void)
{
  // Initialize gpio pins.
  dht11Pins_h = PIN_open(&dht11Pins_s, dht11PinsOutCfg);
 
  //DHT11_Rst();  //��λDHT11
  //DHT11_Check();//�ȴ�DHT11�Ļ�Ӧ
}

/*********************************************************************
 * @fn      DHT11_PIN_SetOUT
 *
 * @brief   DHT11_PIN_SetOUT
 *
 * @param   value
 *
 * @return  none
 */
void DHT11_PIN_SetOUT(uint8_t value)
{
#ifdef USE_FIRST_METHOD
  //PIN_setConfig(dht11Pins_h, PIN_BM_INPUT_EN, DHT11_PIN|PIN_INPUT_DIS); 
  //PIN_setConfig(dht11Pins_h, PIN_BM_GPIO_OUTPUT_EN, DHT11_PIN|PIN_GPIO_OUTPUT_EN|PIN_PUSHPULL);
  //PIN_setOutputEnable(dht11Pins_h,DHT11_PIN,true);
  PIN_setOutputValue(dht11Pins_h,DHT11_PIN,value);
#else
  PIN_setOutputValue(dht11Pins_h,DHT11_PIN,value);
#endif
}

/*********************************************************************
 * @fn      DHT11_PIN_GetIN
 *
 * @brief   DHT11_PIN_GetIN
 *
 * @param   none
 *
 * @return  value
 */
uint8_t DHT11_PIN_GetIN()
{
#ifdef USE_FIRST_METHOD
  //PIN_setConfig(dht11Pins_h, PIN_BM_GPIO_OUTPUT_EN, DHT11_PIN|PIN_GPIO_OUTPUT_DIS); 
  //PIN_setConfig(dht11Pins_h, PIN_BM_INPUT_EN, DHT11_PIN|PIN_INPUT_EN);
  //PIN_setOutputEnable(dht11Pins_h,DHT11_PIN,false);
  return PIN_getInputValue(DHT11_PIN);
#else
  //PIN_setConfig(dht11Pins_h, PIN_BM_GPIO_OUTPUT_EN, DHT11_PIN|PIN_GPIO_OUTPUT_DIS); 
  //PIN_setConfig(dht11Pins_h, PIN_BM_INPUT_EN, DHT11_PIN|PIN_INPUT_EN);
  //PIN_setOutputEnable(dht11Pins_h,DHT11_PIN,false);
  return PIN_getInputValue(DHT11_PIN);
#endif
      
    
}


/*********************************************************************
 * @fn      DHT11_Rst
 *
 * @brief   DHT11 Reset
 *
 * @param   none
 *
 * @return  none
 */
void DHT11_Rst(void)
{
  
#if 1
  PIN_setOutputEnable(dht11Pins_h,DHT11_PIN,true);
  DHT11_PIN_SetOUT(LOW); 	//����
  delay_ms(20);    	//��������18ms
  
  DHT11_PIN_SetOUT(HIGH); 	//����
  delay_us(30);     	//��������20~40us
  //delay_10us(3);
#endif
  
  
#if 0
  //dbg delay_us
  while(1)
  {
    DHT11_PIN_SetOUT(HIGH); 	
    delay_us(10);     	 
    DHT11_PIN_SetOUT(LOW); 	
    delay_us(10);     	
  }
#endif
#if 0
  //dbg delay_10us
  while(1)
  {
    DHT11_PIN_SetOUT(HIGH); 	
    delay_10us(1);     	 
    DHT11_PIN_SetOUT(LOW); 	
    delay_10us(1);     	
  }
#endif
#if 0
  //dbg IO INPUT
  Board_UartWrite_Int(DHT11_PIN_GetIN());
#endif
}

/*********************************************************************
 * @fn      DHT11_Check
 *
 * @brief   �ȴ�DHT11�Ļ�Ӧ
 *
 * @param   none
 *
 * @return  0 - yes,  1 - no
 */
uint8_t DHT11_Check(void)
{
   
  uint8_t retry=0;
  PIN_setOutputEnable(dht11Pins_h,DHT11_PIN,false);
  //while ((DHT11_PIN_GetIN() == 1) && retry < 100)//DHT11������40~80us
  while ((DHT11_PIN_GetIN() == 1) && retry < 10)//DHT11������40~80us
  {
     retry++;
     //delay_us(1);
      delay_10us(1);
  }
  
  //if(retry>=100) return 1;
  if(retry>=10) return 1;
  else retry=0;
  //while ((DHT11_PIN_GetIN() == 0) && retry < 100)//DHT11���ͺ���ٴ�����40~80us
  while ((DHT11_PIN_GetIN() == 0) && retry < 10)//DHT11���ͺ���ٴ�����40~80us
  {
    retry++;
    //delay_us(1);
    delay_10us(1);
  }
  
  //if(retry>=100) return 1;	   
  if(retry>=10) return 1;	
  else retry=0;
  
#if 1
  //while ((DHT11_PIN_GetIN() == 0) && retry < 100)//
  while ((DHT11_PIN_GetIN() == 1) && retry < 10)
  {
    retry++;
    //delay_us(1);
    delay_10us(1);
  }
  if(retry>=10) return 1;	
#endif

  return 0;

}
  
/*********************************************************************
 * @fn      temperatureSensorClose
 *
 * @brief   close DHT11 sensor
 *
 * @param   none
 *
 * @return  none
 */
void temperatureSensorClose(void)
{
  PIN_close(dht11Pins_h);
}

/*********************************************************************
 * @fn      DHT11_Read_Bit
 *
* @brief   read one bit data
 *
 * @param   none
 *
 * @return  1/0
 */
uint8_t DHT11_Read_Bit(void) 			 
{
  uint8_t retry=0;

  //ÿһbit���ݶ���50us�͵�ƽʱ϶��ʼ
  //while((DHT11_PIN_GetIN() == 1) && retry < 100)//�ȴ���Ϊ�͵�ƽ /100
  while((DHT11_PIN_GetIN() == 1) && retry < 10)//�ȴ���Ϊ�͵�ƽ /100
  {
    retry++;
    //delay_us(1);
    delay_10us(1);
  }
 
  retry=0;
  //while((DHT11_PIN_GetIN() == 0) && retry < 100)//�ȴ���ߵ�ƽ
  while((DHT11_PIN_GetIN() == 0) && retry < 10)//�ȴ���ߵ�ƽ
  { 
    retry++;
    //delay_us(1);
    delay_10us(1);
  }
  //�ߵ�ƽ�ĳ��̶�������λ��0����1��0����20-30us  1����60-70us��
  //delay_us(40);//�ȴ�40us
  delay_10us(4);
  if(DHT11_PIN_GetIN() == 1)
  {
    while(DHT11_PIN_GetIN() == 1);
    return 1;
  }
  else 
  {
    return 0;	
  }

}

/*********************************************************************
 * @fn      DHT11_Read_Byte
 *
* @brief   read one byte data
 *
 * @param   none
 *
 * @return  dat
 */
uint8_t DHT11_Read_Byte(void)    
{
  uint8_t i,dat;
  uint8_t retry=0;
  dat=0;
  for (i=0;i<8;i++)
  {
    dat<<=1;
#if 0
    dat|=DHT11_Read_Bit();
#else
   
 
  retry=0;
  while((DHT11_PIN_GetIN() == 0) && retry < 100)//�ȴ���ߵ�ƽ
  { 
    retry++;
    delay_us(1);
  }
  
  retry=0;
  while((DHT11_PIN_GetIN() == 1) && retry < 100)//�ȴ���ߵ�ƽ
  { 
    retry++;
    delay_us(1);
  }
  dbgc[no] =retry;
  no++;
  if(retry > 20) dat|=1;
#endif
  }
  return dat;
}

/*********************************************************************
 * @fn      temperatureSensorRead
 *
* @brief   read once DHT11 sensor's value(temp:0~50, humi:20%~90%)
 *
 * @param   dht11_value
 *
 * @return  0 - ok, 1 - err
 */
uint8_t temperatureSensorRead(dht11_data_t* dht11_value)
{
  int ret = 0;
#ifdef UART_PRINT_ENABLE
  printS("\r\nRead DHT11 Info");
#endif
  
  
#ifdef USE_FIRST_METHOD                                                         //���õ�1�ַ���
  
  uint8_t buf[5];
  uint8_t i;
  no=0;
  DHT11_Rst();

  if(DHT11_Check()==0)
  {
    for(i=0;i<5;i++)//��ȡ40λ����
    {
      buf[i]=DHT11_Read_Byte();
    }
#ifdef UART_PRINT_ENABLE
    printV(dht11_val,5);
#endif
    
    if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
    {
      dht11_value->humi_value=buf[0];
      dht11_value->temp_value=buf[2];
    }
    else
    {
      dht11_value->humi_value=0;
      dht11_value->temp_value=0;
    }
  }
  else 
  {
    dht11_value->humi_value=0;
    dht11_value->temp_value=0;
    ret = -1;
  }	
  
#else                                                                           //���õ�2�ַ���
  
/*
ǰ3�ηֱ��ǣ�1�͵�ƽ��2�ߵ�ƽ������Ӧ�źţ���3�͵�ƽ�������ݵ�һ���͵�ƽ��
i%2==0 ����Ϊÿ�ζ���ѭ����ȡ�͵�ƽ�͸ߵ�ƽ��ÿ��Ҫѭ��2�βŶ���һ��bit����
������j/8��������һ������8��λ������1λ�Զ���0���൱�ڶ���0
counter�����������16����ߵ�ƽ����Ӧ��1.
���ٽ���������1λ��ʹ���һλ���1 
j++8��������һ������
*/
    uint8_t dht11_val[5];
    uint8_t laststate=HIGH;         //last state
    uint8_t counter=0;
    uint8_t j=0,i=0;
    memset(dht11_val,0,5);
    memset(dbgc,0,90);
    //host send start signal  
    PIN_setOutputEnable(dht11Pins_h,DHT11_PIN,true);
    DHT11_PIN_SetOUT(LOW); 	    //set to low at least 18ms 
    delay_ms(18);
    DHT11_PIN_SetOUT(HIGH); 	   //set to high 20-40us
    //delay_10us(2);
    delay_us(15);
     
    PIN_setOutputEnable(dht11Pins_h,DHT11_PIN,false);       //set pin to input
    for(i=0;i<85;i++)         
    {
      counter=0;
      while(DHT11_PIN_GetIN()== laststate){     //read pin state to see if dht responsed. if dht always high for 255 + 1 times, break this while circle
        counter++;
        delay_us(1);
        if(counter==255)
          break;
      }
      laststate=DHT11_PIN_GetIN();             //read current state and store as last state. 
      if(counter==255)                            //if dht always high for 255 + 1 times, break this for circle
        break;
      // top 3 transistions are ignored, maybe aim to wait for dht finish response signal
      if((i>=4)&&(i%2==0)){
        dbgc[j] = counter;
        dht11_val[j/8]<<=1;                     //write 1 bit to 0 by moving left (auto add 0)
        if(counter>16)
        {
          //long mean 1
          dht11_val[j/8]|=1;                  //write 1 bit to 1 
        }
        j++;
        //dbgc[i] = counter;
      }    
    }
    dbgc[j] = j;
#if 1
#ifdef UART_PRINT_ENABLE
    printV(dht11_val,5);
#endif
#endif
    // verify checksum 
    //if((j>=40)&&(dht11_val[4]==((dht11_val[0]+dht11_val[1]+dht11_val[2]+dht11_val[3])& 0xFF)))
    if((dht11_val[4]==((dht11_val[0]+dht11_val[1]+dht11_val[2]+dht11_val[3])& 0xFF)))
    {
      dht11_value->humi_value = dht11_val[0];
      dht11_value->temp_value = dht11_val[2];
    }
    else if(j >= 37)
    {//ͨ���������Է��Ͷ��������ȡ������5��6bit���ݣ�����֮��������������2λ
      if(j == 37) i = 3;
      else if(j == 38) i = 2;
      else if(j == 39) i = 1;
      //�ȰѺ�����������iλ,�õ��¶� dht11_val[2],crc dht11_val[4] 
      j = 16;
      for(j = 16; j<40; j++)
      {
        dht11_val[j/8]<<=1;
        dht11_val[j/8]|=((dbgc[j-i]) > 16 ? 1 : 0);
      }
      //��crc���¶ȵõ�ʪ��
      if((dht11_val[4]==((dht11_val[0]+dht11_val[2])& 0xFF)))
      {//�п�����ʪ�����������������м�λû�ж�ȡ��
        dht11_value->humi_value = dht11_val[0];
      }
      else
      {
        dht11_value->humi_value = dht11_val[4] - dht11_val[2];
      }
      dht11_value->temp_value = dht11_val[2];
    }
    else
    {
      dht11_value->humi_value = 255;
      dht11_value->temp_value = 255;
      ret = -1;
    }
#endif
    
#if 0 //def UART_PRINT_ENABLE
  uint8_t tmp[32];
  memset(tmp,0,32);
  sprintf(tmp,"temp:%d",dht11_value->temp_value);
  printS(tmp);

  memset(tmp,0,32);
  sprintf(tmp,"humi:%d",dht11_value->humi_value);
  printS(tmp);
  
  //printV(dbgc,90);
  
#endif
    
  return ret;

}

int last_temp = 0;
int last_humi = 0;
uint8_t getTemperature(dht11_data_t* dht11_value)
{
  int i = 0, no = 0,ret = -1;
  
  //�ɼ�2������ֵ�Ƚ� ��һ�� �����ϱ�
  
  ret = temperatureSensorRead(dht11_value);
  if(ret == -1)
  {
    return ret;
  }
  else
  {
    if(dht11_value->temp_value == 0 && dht11_value->humi_value == 0)
    {
      return ret;
    }
    else
    {
      if(last_temp != dht11_value->temp_value && last_humi != dht11_value->humi_value)
      {
        //report
        RetartTxNextPacketTimer(true,100);
      }
      last_temp = dht11_value->temp_value;
      last_humi = dht11_value->humi_value;
     
      ret = 0;
    }
  }
    
  
  return ret;
}

uint8_t getTsensorValue(dht11_data_t* dht11_value)
{
    if(last_temp != 0 && last_humi != 0)
    {
      dht11_value->temp_value = last_temp;
      dht11_value->humi_value = last_humi;
    }
    else
    {
      getTemperature(dht11_value);
    }
    
    return 0;
  
}

/*********************************************************************
 * @fn      App_Control_Opera_Service
 *
 * @brief   app control service
 *
 * @param   opera
 *
 * @return  none
 */
void App_Control_Opera_Service(int opera)
{
  
  
}