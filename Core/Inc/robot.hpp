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
  }

  void robot_init()
  {
    for (int i = 0; i < 4; i++)
    {
      // Order: 1, 4, 2, 3
      motor_drivers[i] = Driver(motor_dir_ports[i], motor_dir_pins[i], motor_pwm_timers[i], motor_pwm_timer_channels[i], max_pwm[i], 1);
      motor_encoders[i] = Encoder(encoder_timers[i], count_per_revolution[i]);
      pid_controllers[i] = PID(pid_inputs + i, pid_outputs + i, pid_set_points + i, kp[i], ki[i], kd[i], P_ON_E, DIRECT); // passing pointer of pid_inputs, pid_outputs  and pid_set_points

      pid_controllers[i].SetOutputLimits(0, max_motor_omegas[i]);
      pid_controllers[i].SetSampleTime(30);
    }
  }
  Kinematics kinematics = Kinematics(base_radius, wheel_radius);
  Driver motor_drivers[4];
  Encoder motor_encoders[4];
  PID pid_controllers[4];

  float kin_to_perc(float k)
  {
    if (k < 0)
      k = k * (-1.0);
    return (10.0f * k);
  }

  inline GPIO_PinState get_dir(float omega)
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

    kinematics.get_motor_omegas(1.5f, 0.0f, 0.0f); // TODO: here we have to use Vx, Vy and omega from joystick
    for (int i = 0; i < 4; i++)
    {
      pid_inputs[i] = motor_encoders[i].get_encoder_omega();
    }

    pid_set_points[0] = kinematics.v1;
    pid_set_points[1] = kinematics.v2;
    pid_set_points[2] = kinematics.v3;
    pid_set_points[3] = kinematics.v4;

    if ((
            pid_controllers[0].Compute() &&
            pid_controllers[1].Compute() &&
            pid_controllers[2].Compute() &&
            pid_controllers[3].Compute()))
    {
    }
    for (int i = 0; i < 4; i++)
    {
      motor_drivers[i].run_motor(get_dir(pid_inputs[i]), pid_outputs[i]);
    }
    prev_tim = HAL_GetTick();
  }
};

#endif // __cplusplus

void init_robot()
{

  Robot r;
  while (1)
  {
    HAL_GPIO_TogglePin(M1D_GPIO_Port, M2D_Pin);
    r.run_tick();
  }
}