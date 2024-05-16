#ifdef _JOY_MSG_HPP
#define _JOY_MSG_HPP
#endif

#include <memory.h>
#include "usart.h"
//#include <gpio.c>
//#include "crc8.hpp"


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
