#include "robot.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

void init_robot() { printf("Init robot\n"); }

void operate_robot() {
  while (true) {
    printf("Operate_robot\n");
    HAL_Delay(100);
  }
}
