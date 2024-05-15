//#ifndef ROBOT_H_
//#define ROBOT_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "tim.h"
#include "driver.hpp"
#include "gpio.h"
#include "api_functions.hpp"
#include "kinematics.hpp"

#ifdef __cplusplus

class Robot {
 private:
  float base_radius = 0.0f;
  float wheel_radius = 0.0f;
  float motor_omegas[4];

 public:

  GPIO_TypeDef* motor_dir_ports[4] = {M1D_GPIO_Port,M2D_GPIO_Port,M3D_GPIO_Port,M4D_GPIO_Port}; // for direction
  uint16_t motor_dir_pins[4] = {M1D_Pin,M2D_Pin,M3D_Pin,M4D_Pin};

  TIM_HandleTypeDef* motor_pwm_timers[4] = {&M1P_Timer,&M2P_Timer,&M3P_Timer,&M4P_Timer}; // for pwm timer
  uint32_t motor_pwm_timer_channels[4] = {M1P_Tim_Channel,M2P_Tim_Channel,M3P_Tim_Channel,M4P_Tim_Channel};

  TIM_HandleTypeDef* encoder_timers[4] = {&ENC1_Timer,&ENC2_Timer,&ENC3_Timer,&ENC5_Timer}; // for encoder timer

  float max_pwm[4] = {499,499,499,499};

  Kinematics kinematics = Kinematics(1.0,1.0);



  
  Driver m1_driver = Driver(motor_dir_ports[0],motor_dir_pins[0],motor_pwm_timers[0],motor_pwm_timer_channels[0],max_pwm[0],1);
  Driver m2_driver = Driver(motor_dir_ports[1],motor_dir_pins[1],motor_pwm_timers[1],motor_pwm_timer_channels[1],max_pwm[1],1);
  Driver m3_driver = Driver(motor_dir_ports[2],motor_dir_pins[2],motor_pwm_timers[2],motor_pwm_timer_channels[2],max_pwm[2],1);
  Driver m4_driver = Driver(motor_dir_ports[3],motor_dir_pins[3],motor_pwm_timers[3],motor_pwm_timer_channels[3],max_pwm[3],1);

  Robot() {
    
  }


  float kin_to_perc(float k) {
    return (1.0 * k);
  }
  
  void run_tick() {

    kinematics.set_value(50, 45, 0);


    m1_driver.run_motor(kin_to_perc(kinematics.v0), GPIO_PIN_SET);
    m1_driver.run_motor(kin_to_perc(kinematics.v0), GPIO_PIN_SET);
    m1_driver.run_motor(kin_to_perc(kinematics.v0), GPIO_PIN_SET);
    m1_driver.run_motor(kin_to_perc(kinematics.v0), GPIO_PIN_SET);
  }
};

#endif // __cplusplus


void init_robot() {

 Robot r;
 while (1) {
   r.run_tick();
 }
}

void operate_robot() {

}
