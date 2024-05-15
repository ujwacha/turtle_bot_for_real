#include <cstdint>
#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_tim.h"
class Driver{
  public:
  Driver(GPIO_TypeDef* dchannel, uint16_t dpin, TIM_HandleTypeDef* tim_pwm, uint16_t channel, int m_pwm);
  void run_motor(float percentage, int dir);

  private:
  GPIO_TypeDef* dir_channel;
  uint16_t dir_pin;

  TIM_HandleTypeDef* timer_pwm;
  uint16_t timer_channel;
  int max_pwm;
};
