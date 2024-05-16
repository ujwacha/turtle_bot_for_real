#include <memory.h>
#include "joystick.hpp" 
#include "usart.h"
#include "gpio.c"
#include "crc8.hpp"


#ifdef _JOY_MSG_HPP
#define _JOY_MSG_HPP

#define START_BYTE 0xA5


#define BUTTONS(button) (1 << button)
namespace JoyMsg
{
    namespace PS4Msg
    {
        enum PS4Buttons
        {
             Cross,
             Circle,
             Square,
             Triangle,
             Share,
             Power,
             Option,
             L3,
             R3,
             L1, 
             R1,
             Up,
             Down,
             Left,
             Right,
             Touch
        };
    }  
}     
 struct JoyData
{
  int8_t lx;
  int8_t ly;
  int8_t rx;
  int8_t ry;
  uint8_t lt;
  uint8_t rt;
  uint16_t buttons;
};

bool is_waiting_for_start_byte = true;
uint8_t data[66];
uint8_t crc_table[CRC8_TABLE_SIZE];


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