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
#include "PID.hpp"
#include "joystick.hpp"
#ifdef __cplusplus
#include <cstdint>
#include <cstdio>




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
  
  init_crc_joy();

  HAL_GPIO_TogglePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin);
  
  }

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

  float omegas[4];

  void robot_init()
  {
   for (int i = 0; i < 4; i++)
   {
	pid_controllers[i].SetOutputLimits(0, max_motor_omegas[i]);
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


   HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
   
   // kinematics.get_motor_omegas(1.5f, 0.0f, 0.0f); // TODO: here we have to use Vx, Vy and omega from joystick
   // for (int i = 0; i < 4; i++)
   // {
   // 	pid_inputs[i] = motor_encoders[i].get_encoder_omega();
   // }

   // pid_set_points[0] = kinematics.v1;
   // pid_set_points[1] = kinematics.v2;
   // pid_set_points[2] = kinematics.v3;
   // pid_set_points[3] = kinematics.v4;

   // if ((
   // 	  pid_controllers[0].Compute() &&
   // 	  pid_controllers[1].Compute() &&
   // 	  pid_controllers[2].Compute() &&
   // 	  pid_controllers[3].Compute()))
   // {
   // }
   // for (int i = 0; i < 4; i++)
   // {
   // 	motor_drivers[i].run_motor(get_dir(pid_inputs[i]), pid_outputs[i]);
   // }

   JoyData data = get_present_data();




   float vy = -1* ((float)data.lx - 127.0f) / 127.0f;
   float vx = -1*((float)data.ly - 127.0f) / 127.0f;
   float om = ((float)(data.r2 - data.l2)) / 255.0f;



   //   // printf("%f %f %f\n", vx, vy, om);
   
   kinematics.get_motor_omegas(vx, vy, om);


   omegas[0] = (float)kinematics.v1;
   omegas[3] = (float)kinematics.v2;
   omegas[1] = (float)kinematics.v3;
   omegas[2] = (float)kinematics.v4;


   for (int i = 0; i < 4; i++) {
     motor_drivers[i].run_motor(get_dir(omegas[i]), kin_to_perc((float)omegas[i]));
     //printf("%f  ", omegas[0]);

   }
   //printf("\n");

   
   //printf("%i  %i  %i  %i\n", data.lx, data.ly, data.l2, data.r2);
   
   // printf("tick: %d %d %d %d %u %u %04x\n",
   // 	  data.lx,
   // 	  data.ly,
   // 	  data.rx,
   // 	  data.ry,
   // 	  data.lt,
   // 	  data.rt,
   // 	  data.buttons);


   // printf("69\n");

   prev_tim = HAL_GetTick();
  }
};

#endif // __cplusplus


void init_robot()
{

 Robot r = Robot();
 while (1)
 {
   //  HAL_GPIO_TogglePin(M1D_GPIO_Port, M2D_Pin);
  r.run_tick();
 }
}
