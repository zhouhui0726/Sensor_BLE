/******************************************************************************
* @file  UartDataProcess.c
*
*****************************************************************************/

#if defined (NPI_USE_UART) || defined (NPI_USE_SPI) 
/*********************************************************************
* INCLUDES
*/
#include "hal_mcu.h"
#include "bcomdef.h"
#include "JC_GATTprofile.h"
#include "osal_snv.h"

#include "UartDataProcess.h"
#include "AskCrc.h"
#include "lorawan_node.h"
/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* MACROS
*/

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
extern uint8_t is_joined;
extern uint8_t advertData[31];
extern uint32_t LoRaMacState;
//#pragma optimize=none
void UartDataProcess(uint8_t *data, uint8_t len)
{
  uint8_t Serial_number_tmp[9];
  uint8_t *Txbuf = (uint8_t *)malloc(len);
  uint8_t status = SUCCESS;
  if(data[2] != len) return;
  if (data[len-1] != (CRC8_Generate(data, len-1) ^ 0xFF)) return;
  if((data[0] << 8 | data[1]) != UART_HEAD_DATA) return;
  
  switch (data[3])
	{
		//########## Advertisement data coming in
		case UART_CMD_WRITE_SERIAL_NUMBER:
      {
        if ((len - 5) != 8) return;
        memcpy(Serial_number_tmp, &data[4], sizeof(Serial_number_tmp));
#ifdef USE_MD5
        if (memcmp(Serial_number, Serial_number_tmp, 8) != 0) {
          Serial_number_tmp[8] = CRC8_Generate(Serial_number_tmp, 8);
          memcpy(Serial_number, Serial_number_tmp, sizeof(Serial_number_tmp));
          status = osal_snv_write(BLE_NVID_USER_SERIAL_NUMBER, sizeof(Serial_number_tmp), Serial_number_tmp);
        }
#endif
        if (status == SUCCESS) {
          memcpy(Txbuf, data, len);
          memset(Serial_number_tmp, 0, sizeof(Serial_number_tmp));
          osal_snv_read(BLE_NVID_USER_SERIAL_NUMBER, sizeof(Serial_number_tmp), Serial_number_tmp);
          memcpy(&Txbuf[4], Serial_number_tmp, 8);
          Txbuf[3] |= 0x80;
          Txbuf[len-1] = (CRC8_Generate(Txbuf, len-1) ^ 0xFF);
          TLwrite(Txbuf, len);
        }
      }
    break;
    
    default:
			break;
  }
  free(Txbuf);
}
//#pragma optimize=none
bool Uart_GeneralModuleDataProcess(uint8_t* data, uint8_t len)
{
  uint8_t *datacache = data;
  bool stat = true;
  switch (datacache[0])
	{
  case 0xC1:
    if (datacache[1] == 0xC1 && datacache[2] == 0xC1 && len == 3) {
      UartReturnModuleInfo *uart_send_data = malloc(sizeof(UartReturnModuleInfo));
      uart_send_data->cmd = 0xC1;
      uart_send_data->networkstat = is_joined;
      memcpy(uart_send_data->version, Get_ModuleVersion(), 4);
      memcpy(uart_send_data->number, Serial_number, 8);
      TLwrite((uint8_t *)uart_send_data, sizeof(UartReturnModuleInfo));
      free(uart_send_data);
    }
    break;
  case 0xC2:
    if (datacache[1] == 0xC2 && datacache[2] == 0xC2 && len == 3) {
      HAL_SYSTEM_RESET();
    }
    break;
  case 0xC3:
    if (is_joined) {
      if (LoRaMacState == 0) {
        LoraWan_SendData(true, 2, len, datacache);
        //send ok
        TLwrite("OK", 2);
      } else {
        //send busy
        TLwrite("BUSY", 4);
      }
    } else {
      //send DISCONNECTED
      TLwrite("DISCONNECTED", 12);
    }
    break;
  case 0xC5:
    TLwrite(datacache, len);
    break;
  default:
    break;
  }
  return stat;
}
#endif