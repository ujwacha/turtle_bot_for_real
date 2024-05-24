#pragma once
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
#include "crc.h"
#include "joystick.h"
#ifdef __cplusplus
#include <cstdint>
#include <cstdio>

template <typename t>
inline t map(t x, t in_min, t in_max, t out_min, t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class Robot
{
 private:
  // for the dimensions of the motor and the base
  static float base_radius;
  static float wheel_radius;
  static uint32_t prev_tim;

  // for the direction pins and ports
  static GPIO_TypeDef *motor_dir_ports[4];
  static uint16_t motor_dir_pins[4];

  // for timer instance and timer channels
  static TIM_HandleTypeDef *motor_pwm_timers[4];
  static uint32_t motor_pwm_timer_channels[4];

  // for encoder timer & total count in one revolution
  static TIM_HandleTypeDef *encoder_timers[4];
  static uint16_t count_per_revolution[4];

  static float max_pwm[4]; // this is the value of pwm for 100% duty cycle
  static float motor_omegas[4];

  double pid_inputs[4];
  double pid_outputs[4];
  double pid_set_points[4];

  float max_motor_omegas[4];

  double kp[4];
  double ki[4];
  double kd[4];

 public:
  Robot() {
   robot_init();
   init_CRC_Table(crc_table, 7); }


  Driver motor_drivers[4] ={
   Driver(motor_dir_ports[0], motor_dir_pins[0], motor_pwm_timers[0], motor_pwm_timer_channels[0], max_pwm[0], 1),
   Driver(motor_dir_ports[3], motor_dir_pins[3], motor_pwm_timers[3], motor_pwm_timer_channels[3], max_pwm[3], 1),
   Driver(motor_dir_ports[1], motor_dir_pins[1], motor_pwm_timers[1], motor_pwm_timer_channels[1], max_pwm[1], 1),
   Driver(motor_dir_ports[2], motor_dir_pins[2], motor_pwm_timers[2], motor_pwm_timer_channels[2], max_pwm[2], 1)
  } ;
  Encoder motor_encoders[4] = {
   Encoder(encoder_timers[0], count_per_revolution[0],10),
   Encoder(encoder_timers[3], count_per_revolution[3],10),
   Encoder(encoder_timers[1], count_per_revolution[1],10),
   Encoder(encoder_timers[2], count_per_revolution[2],10),
  };
  PID pid_controllers[4] = {
   PID(pid_inputs + 0, pid_outputs + 0, pid_set_points + 0, kp[0], ki[0], kd[0], P_ON_E, DIRECT),
   PID(pid_inputs + 3, pid_outputs + 3, pid_set_points + 3, kp[3], ki[3], kd[3], P_ON_E, DIRECT),
   PID(pid_inputs + 1, pid_outputs + 1, pid_set_points + 1, kp[1], ki[1], kd[1], P_ON_E, DIRECT),
   PID(pid_inputs + 2, pid_outputs + 2, pid_set_points + 2, kp[2], ki[2], kd[2], P_ON_E, DIRECT),
  };

  void robot_init()
  {
   for (int i = 0; i < 4; i++)
   {
	pid_controllers[i].SetOutputLimits(-max_motor_omegas[i], max_motor_omegas[i]);
	pid_controllers[i].SetSampleTime(30);
   }
  }

  Kinematics kinematics = Kinematics(base_radius, wheel_radius);

  float kin_to_perc(float k)
  {
   if (k < 0)
	k = k * (-1.0);
   return (10.0f * k);
  }

  GPIO_PinState get_dir(float omega)
  {
   if (omega <= 0)
	return GPIO_PIN_SET;
   else
	return GPIO_PIN_RESET;
  }

  void run_tick()
  {

   if (HAL_GetTick() - prev_tim < 30)
	return;

   float lx = map<float>((float)data.lx, -128.0, 127.0, -1.0, 1.0);
   float ly = map<float>((float)data.ly, -128.0, 127.0, -1.0, 1.0);
   float trig = map<float>((float)(data.lt - data.rt), -255.0, 255.0, -1.0, 1.0);

   kinematics.get_motor_omegas(lx, ly, trig); // TODO: here we have to use Vx, Vy and omega from joystick

   for (int i = 0; i < 4; i++)
   {
	pid_inputs[i] = motor_encoders[i].get_encoder_omega();
   }

   pid_set_points[0] = kinematics.v1 * max_motor_omegas[0];
   pid_set_points[1] = kinematics.v2 * max_motor_omegas[1];
   pid_set_points[2] = kinematics.v3 * max_motor_omegas[2];
   pid_set_points[3] = kinematics.v4 * max_motor_omegas[3];

   if ((
	  pid_controllers[0].Compute() &&
	  pid_controllers[1].Compute() &&
	  pid_controllers[2].Compute() &&
	  pid_controllers[3].Compute()))
   {
   }
   for (int i = 0; i < 4; i++)
   {
   for (int i = 0; i < 4; i++) {
     motor_drivers[i].run_motor(get_dir(pid_inputs[i]), (pid_outputs[i] / max_motor_omegas[i]) * 100.0);
   }
   }
   prev_tim = HAL_GetTick();
  }
};

#endif // __cplusplus


void init_robot()
{

 Robot r = Robot();
 while (1)
 {
  HAL_GPIO_TogglePin(M1D_GPIO_Port, M2D_Pin);
  r.run_tick();
 }
}
