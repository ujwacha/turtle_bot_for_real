#include "robot.hpp"
#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include <cstdint>
#include <stdio.h>

Robot robot = Robot();

void init_robot() { /*printf("Init robot\n");*/ }

void operate_robot() {
 while (true) {
  //printf("Operate_robot\n");
  HAL_Delay(100);
 }
}
