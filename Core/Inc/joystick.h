#include "usart.h"
#include "crc.h"
#include <stdbool.h>

#define START_BYTE 0xA5

#pragma pack(push, 1)

typedef struct
{
  int8_t lx;
  int8_t ly;
  int8_t rx;
  int8_t ry;
  uint8_t lt;
  uint8_t rt;
  uint16_t buttons;
} JoyData;

#pragma pack(pop)

bool is_waiting_for_start_byte = true;
uint8_t Rx_data[10];
uint8_t crc_table[CRC8_TABLE_SIZE];
JoyData data;
uint32_t last_blick = 0;