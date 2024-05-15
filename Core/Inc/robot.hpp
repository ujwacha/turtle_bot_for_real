<<<<<<< HEAD
//#ifndef ROBOT_H_
//#define ROBOT_H_
#include "stm32f4xx.h"
#include "tim.h"
#include "driver.hpp"
#include "gpio.h"
#include "api_functions.hpp"

#ifdef __cplusplus

class Robot {
 private:
  float base_radius = 0.0f;
  float wheel_radius = 0.0f;
  float motor_omegas[4];


 public:
  // Driver m1_driver;
  // Driver m2_driver;
  // Driver m3_driver;
  // Driver m4_driver;

  // GPIO_TypeDef* motor_dir_ports[4] = {M1D_GPIO_Port,M2D_GPIO_Port,M3D_GPIO_Port,M4D_GPIO_Port}; // for direction
  // uint16_t motor_dir_pins[4] = {M1D_Pin,M2D_Pin,M3D_Pin,M4D_Pin};

  // TIM_HandleTypeDef* motor_pwm_timers[4] = {&M1P_Timer,&M2P_Timer,&M3P_Timer,&M4P_Timer}; // for pwm timer
  // uint32_t motor_pwm_timer_channels[4] = {M1P_Tim_Channel,M2P_Tim_Channel,M3P_Tim_Channel,M4P_Tim_Channel};

  // TIM_HandleTypeDef* encoder_timers[4] = {&ENC1_Timer,&ENC2_Timer,&ENC3_Timer,&ENC5_Timer}; // for encoder timer

  float max_pwm[4] = {0.00f,0.00f,0.00f,0.00f};

  //Robot() {}
  void init() {}
  void run() {}
};

#endif // __cplusplus


void init_robot() {

  Robot r;
  while (1) {
    
  }
}

void operate_robot() {
  
}
