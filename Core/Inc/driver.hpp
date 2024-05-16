#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"

class Driver    {
  public:
  Driver(GPIO_TypeDef* dchannel, uint16_t dpin, TIM_HandleTypeDef* tim_pwm, uint32_t channel, int m_pwm, int8_t pdir);
  void run_motor(GPIO_PinState dir,float percentage);


 private:
  GPIO_PinState get_pinstate(GPIO_PinState current); // sets the positive direction of spin for the motor

  GPIO_TypeDef* dir_channel; // dicection pin channel
  uint16_t dir_pin; // direction pin



  TIM_HandleTypeDef* timer_pwm; // timer handler to set pwm
  uint32_t timer_channel; // channel for the pwm
  int max_pwm; // max pwm value
  int8_t pos_dir; // variable to store positive 
};
