#include "driver.hpp"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "gpio.h"

Driver::Driver(GPIO_TypeDef* dchannel, uint16_t dpin, TIM_HandleTypeDef* tim_pwm, uint32_t channel, int m_pwm, int8_t pdir) {
    dir_channel = dchannel;
    dir_pin = dpin;
    timer_channel = channel;
    timer_pwm = tim_pwm;
    max_pwm = m_pwm;
    pos_dir = pdir;
  }

void Driver::run_motor(float percentage, GPIO_PinState dir) {

    uint32_t pwm = (percentage / 100) * max_pwm;
    
    HAL_GPIO_WritePin(dir_channel, dir_pin, get_pinstate(dir));

    __HAL_TIM_SET_COMPARE(timer_pwm, timer_channel, pwm);
}

GPIO_PinState Driver::get_pinstate(GPIO_PinState current) {
  GPIO_PinState retval = current;
  if (pos_dir < 0) {


    switch (current) {
    case GPIO_PIN_RESET:
      retval = GPIO_PIN_SET;
      break;

    case GPIO_PIN_SET:
      retval = GPIO_PIN_RESET;
      break;
    }

  }

  return retval;
}
