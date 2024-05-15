#include "driver.hpp"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"

//=========================================================================================================
//TODO:
//to be suppllied by PID.
enum
{
 clockwise, 
 anticlockwise
};
//=========================================================================================================

Driver::Driver(GPIO_TypeDef* dport, uint16_t dpin, TIM_HandleTypeDef* tim_pwm, uint32_t pwm_timer_channel, int m_pwm) {
    dir_channel = dport;
    dir_pin = dpin;
    timer_channel = channel;
    timer_pwm = tim_pwm;
    max_pwm = m_pwm;
  }

void Driver::run_motor(float percentage, int dir) {
 if(dir == clockwise) HAL_GPIO_WritePin(dir_channel,dir_pin,GPIO_PIN_SET);
 if(dir == anticlockwise) HAL_GPIO_WritePin(dir_channel, dir_pin, GPIO_PIN_RESET);

	uint32_t pwm = (percentage / 100) * max_pwm;
    __HAL_TIM_SET_COMPARE(timer_pwm, timer_channel, pwm);
}
