#ifndef ROBOT_H_
#define ROBOT_H_
#endif
#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "driver.hpp"
#include "gpio.h"
#include "encoder.hpp"
#include "kinematics.hpp"
#include "api_functions.hpp"
#include "kinematics.hpp"
#include "PID.h"
//#include "crc8.hpp"
//#include "joystick.hpp"
#ifdef __cplusplus
#include <cstdint>
#include <cstdio>


class Robot {
 private:
  // for the dimensions of the motor and the base
  static float base_radius; 
  static float wheel_radius;
  static uint32_t prev_tim;

  // for the direction pins and ports
  static GPIO_TypeDef* motor_dir_ports[4];
  static uint16_t motor_dir_pins[4];

  // for timer instance and timer channels
  static TIM_HandleTypeDef* motor_pwm_timers[4];
  static uint32_t motor_pwm_timer_channels[4];

  // for encoder timer & total count in one revolution
  static TIM_HandleTypeDef* encoder_timers[4];
  static uint16_t count_per_revolution[4];

  static float max_pwm[4] ; // this is the value of pwm for 100% duty cycle
  static float motor_omegas[4];

  // static PID motor_controllers[4];

 public:

  Kinematics kinematics = Kinematics(base_radius,wheel_radius);

  Driver m1_driver = Driver(motor_dir_ports[0],motor_dir_pins[0],motor_pwm_timers[0],motor_pwm_timer_channels[0],max_pwm[0],1);
  Driver m2_driver = Driver(motor_dir_ports[2],motor_dir_pins[2],motor_pwm_timers[2],motor_pwm_timer_channels[2],max_pwm[2],1);
  Driver m3_driver = Driver(motor_dir_ports[3],motor_dir_pins[3],motor_pwm_timers[3],motor_pwm_timer_channels[3],max_pwm[3],1);
  Driver m4_driver = Driver(motor_dir_ports[1],motor_dir_pins[1],motor_pwm_timers[1],motor_pwm_timer_channels[1],max_pwm[1],1);

  Encoder m1_encoder = Encoder(encoder_timers[0],count_per_revolution[0]);
  Encoder m2_encoder = Encoder(encoder_timers[2],count_per_revolution[2]);
  Encoder m3_encoder = Encoder(encoder_timers[3],count_per_revolution[3]);
  Encoder m4_encoder = Encoder(encoder_timers[1],count_per_revolution[1]);



  // PID m2_encoder = 
  // PID m3_encoder = 
  // PID m4_encoder = 



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
   //prev_tim = HAL_GetTick();
   //   while ((HAL_GetTick()-prev_tim) <= 3000)   {
   m1_driver.run_motor(get_dir(m1_vel), kin_to_perc(m1_vel)); 
   m2_driver.run_motor(get_dir(m2_vel), kin_to_perc(m2_vel)); 
   m3_driver.run_motor(get_dir(m3_vel), kin_to_perc(m3_vel)); 
   m4_driver.run_motor(get_dir(m4_vel), kin_to_perc(m4_vel)); 
   //m1_driver.run_motor(GPIO_PIN_RESET,10.0f); // this is perfectly good
	//m2_driver.run_motor(GPIO_PIN_RESET,10.0f);//this runs motor 3
	//m3_driver.run_motor(GPIO_PIN_RESET,10.0f);
	//m4_driver.run_motor(GPIO_PIN_RESET,10.0f);//this runs motor 2
	//   }
	//   kinematics.reset();
  }

  void run_tick() {


   // kinematics.get_motor_omegas(0.4f,0.0f, 0.0f);
   // temp_run();
   // HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);

   // kinematics.get_motor_omegas(0.0f,0.4f, 0.0f);
   // temp_run();
   // HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);

   // kinematics.get_motor_omegas(-0.4f,0.0f, 0.0f);
   // temp_run();
   // HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);

   // kinematics.get_motor_omegas(0.0f,-0.4f, 0.0f);
   // temp_run();
   // HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);


    if (HAL_GetTick() - prev_tim < 30) return;
    
    kinematics.get_motor_omegas(1.5f, 0.0f, 0.0f);
    temp_run();
    kinematics.reset();

    // int32_t count = encoder_timers[0]->Instance->CNT;

    // if(count > int32_t(32768))
    //   {
    // 	count = count - int32_t(65536);
    //   }

    float omega = m1_encoder.get_encoder_omega();

    int data = omega * 100;

    printf("%i\n", data);



    prev_tim = HAL_GetTick();

    //  encoder_timers[0]->Instance->CNT = 0;
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
