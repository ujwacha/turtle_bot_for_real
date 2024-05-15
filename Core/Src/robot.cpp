#include "robot.hpp"
#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include <cstdint>
#include <stdio.h>

// Robot robot = Robot();

// robot.m1_driver = Driver(motor_dir_ports[0],motor_dir_pins[0],motor_pwm_timers[0],motor_pwm_timer_chhannels[0],max_pwm[0]);
// robot.m2_driver = Driver(motor_dir_ports[1],motor_dir_pins[1],motor_pwm_timers[1],motor_pwm_timer_channels[1],max_pwm[1]);
// robot.m3_driver = Driver(motor_dir_ports[2],motor_dir_pins[2],motor_pwm_timers[2],motor_pwm_timer_channels[2],max_pwm[2]);
// robot.m4_driver = Driver(motor_dir_ports[3],motor_dir_pins[3],motor_pwm_timers[3],motor_pwm_timer_channels[3],max_pwm[3]);
