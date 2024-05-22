#include "joystick.hpp" 


void blick()
{
 uint32_t now = HAL_GetTick();
 if (now - last_blick > 50)
 {
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  last_blick = now;
 }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 blink();
 printf("Recved\n");

 if(huart-> Instance == huart4.Instance)
 { 
  if (is_waiting_for_start_byte)
  {
   if (Rx_data[0] == START_BYTE)
   {
	is_waiting_for_start_byte = false;
	HAL_UART_Receive_DMA(huart, data, sizeof(data) - 1);
   }
   else
   {

	HAL_UART_Receive_DMA(huart, data, 1);
   }
  }
  else
  {
   is_waiting_for_start_byte = true;
   HAL_UART_Receive_DMA(huart, data, 1);
   uint8_t hash = get_CRC_Hash(data + 1, 12, crc_table);
   if (hash == data[sizeof(data) - 1])
   {
	memcpy(&JoyData,data + 1, 64);

   }

  }
 }
}
