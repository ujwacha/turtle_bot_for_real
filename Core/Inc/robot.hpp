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
#include "PID.hpp"
#include "crc8.hpp"
#include "joystick.hpp"
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

  double pid_inputs[4];
  double pid_outputs[4];
  double pid_set_points[4];

  float max_motor_omegas[4];

  double kp[4];
  double ki[4];
  double kd[4];

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

  PID Controller1 = PID(pid_inputs+0, pid_outputs+0, pid_set_points+0, kp[0], ki[0], kd[0], P_ON_E, DIRECT); // passing pointer of pid_inputs, pid_outputs  and pid_set_points
  PID Controller2 = PID(pid_inputs+1, pid_outputs+1, pid_set_points+1, kp[1], ki[1], kd[1], P_ON_E, DIRECT);// passing pointer of pid_inputs, pid_outputs  and pid_set_points
  PID Controller3 = PID(pid_inputs+2, pid_outputs+2, pid_set_points+2, kp[2], ki[2], kd[2], P_ON_E, DIRECT);// passing pointer of pid_inputs, pid_outputs  and pid_set_points
  PID Controller4 = PID(pid_inputs+3, pid_outputs+3, pid_set_points+3, kp[3], ki[3], kd[3], P_ON_E, DIRECT);// passing pointer of pid_inputs, pid_outputs  and pid_set_points

  void controller_init()
  {
   Controller1.SetOutputLimits(0,max_motor_omegas[0]);
   Controller2.SetOutputLimits(0,max_motor_omegas[1]);
   Controller3.SetOutputLimits(0,max_motor_omegas[2]);
   Controller4.SetOutputLimits(0,max_motor_omegas[3]);

   Controller1.SetSampleTime(30);
   Controller2.SetSampleTime(30);
   Controller3.SetSampleTime(30);
   Controller4.SetSampleTime(30);
  }

  Robot() {
   controller_init();
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

  void run_tick() {

   if (HAL_GetTick() - prev_tim < 30) return;

   kinematics.get_motor_omegas(1.5f, 0.0f, 0.0f); // TODO: here we have to use Vx, Vy and omega from joystick

   pid_inputs[0] = m1_encoder.get_encoder_omega();
   pid_inputs[1] = m1_encoder.get_encoder_omega();
   pid_inputs[2] = m1_encoder.get_encoder_omega();
   pid_inputs[3] = m1_encoder.get_encoder_omega();

   pid_set_points[0] = kinematics.v1;
   pid_set_points[1] = kinematics.v2;
   pid_set_points[2] = kinematics.v3;
   pid_set_points[3] = kinematics.v4;

   if((
	  Controller1.Compute()&&
	  Controller2.Compute()&&
	  Controller3.Compute()&&
	  Controller4.Compute()
	  )) {}

   m1_driver.run_motor(get_dir(pid_inputs[0]), pid_outputs[0]);

   prev_tim = HAL_GetTick();
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
