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




float enc_measurements[4];
float pid_setpoints[4];
float pwm_percts[4];
 
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

  // double pid_inputs[4] = {0.0, 0.0, 0.0, 0.0};
  // double pid_outputs[4] = {0.0, 0.0, 0.0, 0.0};
  // double pid_set_points[4] = {0.0, 0.0, 0.0, 0.0};
 
  float max_motor_pwms[4] = {499.0f, 499.0f, 499.0f, 499.0f}; // pwm

  double kp[4] = {12.0f, 4.0f, 4.0f, 4.0f};
  double ki[4] = {25.0f, 1.0f, 1.0f, 1.0f};
  double kd[4] = {0.00f, 0.0f, 0.0f, 0.0f};


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
    Encoder(encoder_timers[0], count_per_revolution[0],10,1),
    Encoder(encoder_timers[3], count_per_revolution[3],10,-1),
    Encoder(encoder_timers[1], count_per_revolution[1],10,1),
    Encoder(encoder_timers[2], count_per_revolution[2],10,1),
  };
  PID pid_controllers[4] = {
    PID(kp[0], ki[0], kd[0]),
    PID(kp[3], ki[3], kd[3]),
    PID(kp[1], ki[1], kd[1]),
    PID(kp[2], ki[2], kd[2]),
  };

  float omegas[4];

  void robot_init()
  {
   for (int i = 0; i < 4; i++)
   {
	pid_controllers[i].set_sampling_time(0.03);
        pid_controllers[i].set_tau(0.01);
	pid_controllers[i].set_limits(-1 * max_motor_pwms[i], max_motor_pwms[i]);
	pid_controllers[i].set_int_limits(-350.0f, +350.0f);
   }
  }

  Kinematics kinematics = Kinematics(base_radius, wheel_radius);

  float kin_to_perc(float k)
  {
    if (k < 0)
      return k * (-1.0);
    else
      return k;

  }

  float pwm_to_perc(float k) {
    if (k < 0)
      k = k * (-1.0);
    
    if (k > 499) {
      return 100.00f;
    }
    
    return (k / 5.0f);
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

   JoyData data = get_present_data();

   float vy = -1 * ((float)data.lx - 127.0f) / 65.0f;
   
   float vx = -1 * ((float)data.ly - 127.0f) / 65.0f;
   
   float om = ((float)(data.r2 - data.l2)) / 100.0f;


   if (vx < 0.1 && vx > -0.1) {
     vx = 0;
   }

   if (vy < 0.1 && vy > -0.1) {
     vy = 0;
   }

   if (om< 0.1 && om > -0.1) {
     om = 0;
   }
   
   // printf("%f %f %f\n", vx, vy, om);
   
   kinematics.get_motor_omegas(vx, vy, om);


   float k = 10.0f;
   
   omegas[0] = k * (float)kinematics.v1;
   omegas[3] = k * (float)kinematics.v2;
   omegas[1] = k * (float)kinematics.v3;
   omegas[2] = k * (float)kinematics.v4;



   enc_measurements[0] = motor_encoders[0].get_encoder_omega();
   pid_setpoints[0] = om * 20.0f;
   
   pwm_percts[0] =
       pid_controllers[0].PIDController_Update(pid_setpoints[0], enc_measurements[0]);

   motor_drivers[0].run_motor(get_dir(pwm_percts[0]), pwm_to_perc(pwm_percts[0]));
   
  printf("%f %f %f\n", enc_measurements[0], pid_setpoints[0], pwm_to_perc(pwm_percts[0]));
   
   // for (int i = 0; i < 4; i++) {
//      float measurement = motor_encoders[i].get_encoder_omega();
//      float setpoint = omegas[i];
//      float pidout = pid_controllers[i].PIDController_Update(setpoint, measurement);

//      motor_drivers[i].run_motor(get_dir(omegas[i]), kin_to_perc(omegas[i]));

//      printf("(%i , %f, %f, %f, %f) ", get_dir(omegas[i]), kin_to_perc(omegas[i]), pwm_to_perc(pidout), setpoint, measurement);

//      // printf("  (%f , %f)  ", motor_encoders[i].get_encoder_omega(), pidout);

// //     printf("(%i, %f) ", get_dir(pidout), pwm_to_perc(pidout));
//    }

   printf("\n");

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
