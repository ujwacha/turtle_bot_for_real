#include "stm32f4xx_hal.h"

int ITM_SendString(char *data, int len) {
  int i = 0;
  for (; i < len; ++i) {
    ITM_SendChar(data[i]);
  }
  return i;
}

int _write(int file, char *data, int len) {
  int sent = ITM_SendString(data, len);
  return sent;
}
