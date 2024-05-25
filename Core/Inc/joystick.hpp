#include "usart.h"
#include <stdbool.h>
#include <cstring>
#include <cstdio>
#include "crc8.hpp"
#define START_BYTE 0xA5

#pragma pack(push, 1)

typedef struct _JoyData
{
  uint16_t buttons;
  uint8_t lx;
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;
  uint8_t l2;
  uint8_t r2;
  uint8_t hat;
  uint8_t pad;
} JoyData;

#pragma pack(pop)


void init_crc_joy();
JoyData get_present_data();
