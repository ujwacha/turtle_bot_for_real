//#ifndef ROBOT_H_

#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "driver.hpp"
#include "gpio.h"
#include "kinematics.hpp"
#include "api_functions.hpp"
#include "kinematics.hpp"
#include <cstdint>

#ifdef __cplusplus

class Robot {
 private:
  float base_radius = 0.595f/2.0f;
  float wheel_radius = 0.425f/2.0f;
  float motor_omegas[4];
  uint32_t prev_tim;
  // motor 1 : en5_ch1,m1d m1p
  // motor 2 : en3_ch1,m2d, m2p
  // motor 3 : en2_ch1,m3p_m3d
  // motor 4 : en1_ch1,m4p,m4d
 public:

  GPIO_TypeDef* motor_dir_ports[4] = {M1D_GPIO_Port,M2D_GPIO_Port,M3D_GPIO_Port,M4D_GPIO_Port}; // for direction
  uint16_t motor_dir_pins[4] = {M1D_Pin,M2D_Pin,M3D_Pin,M4D_Pin};

  TIM_HandleTypeDef* motor_pwm_timers[4] = {&M1P_Timer,&M2P_Timer,&M3P_Timer,&M4P_Timer}; // for pwm timer
  uint32_t motor_pwm_timer_channels[4] = {M1P_Tim_Channel,M2P_Tim_Channel,M3P_Tim_Channel,M4P_Tim_Channel};

  TIM_HandleTypeDef* encoder_timers[4] = {&ENC5_Timer,&ENC3_Timer,&ENC2_Timer,&ENC1_Timer}; // for encoder timer

  float max_pwm[4] = {499,499,499,499};

  Kinematics kinematics = Kinematics(base_radius,wheel_radius);

  Driver m1_driver = Driver(motor_dir_ports[0],motor_dir_pins[0],motor_pwm_timers[0],motor_pwm_timer_channels[0],max_pwm[0],1);
  Driver m2_driver = Driver(motor_dir_ports[2],motor_dir_pins[2],motor_pwm_timers[2],motor_pwm_timer_channels[2],max_pwm[2],1);
  Driver m3_driver = Driver(motor_dir_ports[3],motor_dir_pins[3],motor_pwm_timers[3],motor_pwm_timer_channels[3],max_pwm[3],1);
  Driver m4_driver = Driver(motor_dir_ports[1],motor_dir_pins[1],motor_pwm_timers[1],motor_pwm_timer_channels[1],max_pwm[1],1);

  Robot() {

  }

  float kin_to_perc(float k) {
   if(k<0) k=k*(-1.0);
   return (10.0f * k);
  }


  inline GPIO_PinState get_dir(float omega)
  {
   if(omega <= 0) return GPIO_PIN_SET;
   else return GPIO_PIN_RESET;
  }

  void temp_run()
  {
   float m1_vel = kinematics.v1;
   float m2_vel = kinematics.v2;
   float m3_vel = kinematics.v3;
   float m4_vel = kinematics.v4;
   prev_tim = HAL_GetTick();
   while ((HAL_GetTick()-prev_tim) <= 3000)   {
	m1_driver.run_motor(get_dir(m1_vel), kin_to_perc(m1_vel)); 
	m2_driver.run_motor(get_dir(m2_vel), kin_to_perc(m2_vel)); 
	m3_driver.run_motor(get_dir(m3_vel), kin_to_perc(m3_vel)); 
	m4_driver.run_motor(get_dir(m4_vel), kin_to_perc(m4_vel)); 
	//m1_driver.run_motor(GPIO_PIN_RESET,10.0f); // this is perfectly good
	//m2_driver.run_motor(GPIO_PIN_RESET,10.0f);//this runs motor 3
	//m3_driver.run_motor(GPIO_PIN_RESET,10.0f);
	//m4_driver.run_motor(GPIO_PIN_RESET,10.0f);//this runs motor 2
   }
   kinematics.reset();
  }

  void run_tick() {
   kinematics.get_motor_omegas(0.4f,0.0f, 0.0f);
   temp_run();
   HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);

   kinematics.get_motor_omegas(0.0f,0.4f, 0.0f);
   temp_run();
   HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);

   kinematics.get_motor_omegas(-0.4f,0.0f, 0.0f);
   temp_run();
   HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);

   kinematics.get_motor_omegas(0.0f,-0.4f, 0.0f);
   temp_run();
   HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);

  }
};

#endif // __cplusplus


void init_robot() {

 Robot r;
 while (1) {
  HAL_GPIO_TogglePin(M1D_GPIO_Port, M2D_Pin);
  r.run_tick();
 }
}


void operate_robot() {

}
