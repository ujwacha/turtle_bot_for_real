#include "robot.hpp"

float Robot::base_radius = 0.595f/2.0f;
float Robot::wheel_radius = 0.425f/2.0f;
uint32_t Robot::prev_tim;

GPIO_TypeDef* Robot::motor_dir_ports[4] = {M1D_GPIO_Port,M2D_GPIO_Port,M3D_GPIO_Port,M4D_GPIO_Port}; // for direction
uint16_t Robot::motor_dir_pins[4] = {M1D_Pin,M2D_Pin,M3D_Pin,M4D_Pin};

TIM_HandleTypeDef* Robot::motor_pwm_timers[4] = {&M1P_Timer,&M2P_Timer,&M3P_Timer,&M4P_Timer}; // for pwm timer
uint32_t Robot::motor_pwm_timer_channels[4] = {M1P_Tim_Channel,M2P_Tim_Channel,M3P_Tim_Channel,M4P_Tim_Channel};

TIM_HandleTypeDef* Robot::encoder_timers[4] = {&ENC5_Timer,&ENC3_Timer,&ENC2_Timer,&ENC1_Timer}; // for encoder timer

float Robot::max_pwm[4] = {499,499,499,499};

uint16_t Robot::count_per_revolution[4] = {1000,1000,1000,500};


