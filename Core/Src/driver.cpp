#include "driver.hpp"
#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "gpio.h"
#include "stm32f4xx_hal_tim.h"
#include <stdio.h>

Driver::Driver(GPIO_TypeDef* dchannel, uint16_t dpin, TIM_HandleTypeDef* tim_pwm_handler, uint32_t channel, int m_pwm, int8_t pdir) {
 dir_channel = dchannel;
 dir_pin = dpin;
 timer_channel = channel;
 timer_pwm = tim_pwm_handler;
 max_pwm = m_pwm;
 pos_dir = pdir;
 HAL_TIM_PWM_Start(timer_pwm,timer_channel);
}

void Driver::run_motor(GPIO_PinState dir,float percentage) {

 uint32_t pwm = (percentage / 100.00f) * max_pwm;
 HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
 // printf("%d\n",(int)percentage);
 HAL_GPIO_WritePin(dir_channel, dir_pin, get_pinstate(dir));
 __HAL_TIM_SET_COMPARE(timer_pwm, timer_channel, pwm);

 //=========================================================
 //this is to remind us of our failure as programmers.
 //
 //=========================================================
 /*timer_pwm->channel = pwm;
   switch (timer_channel) {
   case 1:
   timer_pwm->CCR1 = pwm;
   break;
   case 2:
   timer_pwm->CCR2 = pwm;
   break;
   case 3:
   timer_pwm->CCR3 = pwm;
   break;
   case 4:
   timer_pwm->CCR4 = pwm;
   break;
   }*/

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
