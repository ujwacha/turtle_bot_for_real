#include "driver.hpp"

Driver::Driver(uint32_t dchannel, uint32_t dpin, TIM_HandleTypeDef* tim_pwm, uint32_t channel, int m_pwm) {
    dir_channel = dchannel;
    dir_pin = dpin;
    timer_channel = channel;
    timer_pwm = tim_pwm;
    max_pwm = m_pwm;
  }

void Driver::run_motor(float percentage, int dir) {

    uint32_t pwm = (percentage / 100) * max_pwm;

    __HAL_TIM_SET_COMPARE(timer_pwm, timer_channel, pwm);
}
