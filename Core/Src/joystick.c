 #include "joystick.h"

 void blink()
{
  uint32_t now = HAL_GetTick();
  if (now - last_blick > 50)
  {
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    last_blick = now;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart4.Instance)
  {
    if (is_waiting_for_start_byte)
    {

      if (Rx_data[0] == START_BYTE)
      {
        is_waiting_for_start_byte = false;
        HAL_UART_Receive_DMA(huart, Rx_data + 1, sizeof(Rx_data) - 1);
      }
      else
      {

        HAL_UART_Receive_DMA(huart, Rx_data[0], 1);
      }
    }
    else
    {
      is_waiting_for_start_byte = true;
    

      uint8_t hash = get_CRC_Hash(Rx_data + 1, 8, crc_table);
      if (hash == Rx_data[sizeof(Rx_data) - 1])
      {
          memcpy(&data, Rx_data + 1, 8);
          blink();
          printf("%d %d %d %d %u %u %04x\n",
               data.lx,
               data.ly,
               data.rx,
               data.ry,
               data.lt,
               data.rt,
               data.buttons);
      }
    }
  }
}