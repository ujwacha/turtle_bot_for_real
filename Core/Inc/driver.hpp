#include <cstdint>
#include "main.h"
#include "stm32f4xx_hal_tim.h"
class Driver{
  public:
  Driver(uint32_t dchannel, uint32_t dpin, TIM_HandleTypeDef* tim_pwm, uint32_t channel, int m_pwm);
  void run_motor(float percentage, int dir);


  private:
  uint32_t dir_channel;
  uint32_t dir_pin;

  TIM_HandleTypeDef* timer_pwm;
  uint32_t timer_channel;
  int max_pwm;
};
