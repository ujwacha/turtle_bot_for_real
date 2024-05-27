#include "PID.hpp"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"

PID::PID(float _kp, float _ki, float _kd, float _T, float _tau,
    float _min , float _max , float _intmin ,
    float _intmax ) {
  Kp = _kp;
  Ki = _ki;
  Kd = _kd;

  tau = _tau;

  limMin = _min;
  limMax = _max;

  limMaxInt = _intmax;
  limMinInt = _intmin;

  T = _T;
}

void PID::set_limits(float _min, float _max) {
  limMin = _min;
  limMax = _max;
}

void PID::set_int_limits(float _intmin, float _intmax) {
  limMaxInt = _intmax;
  limMinInt = _intmin;
}

void PID::set_sampling_time(float _T) { T = _T; }

void PID::set_tau(float _tau) { tau = _tau; }

float PID::PIDController_Update(float setpoint, float measurement) {

  if (HAL_GetTick() - prev_time < (T * 1000.0f))
    return out;


  HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

  /*
   * Error signal
   */
  float error = setpoint - measurement;

  /*
   * Proportional
   */
  float proportional = Kp * error;

  /*
   * Integral
   */
  integrator = integrator + 0.5f * Ki * T * (error + prevError);

  /* Anti-wind-up via integrator clamping */
  if (integrator > limMaxInt) {

    integrator = limMaxInt;

  } else if (integrator < limMinInt) {

    integrator = limMinInt;
  }

  /*
   * Derivative (band-limited differentiator)
   */

  differentiator =
      -(2.0f * Kd *
            (measurement -
             prevMeasurement) /* Note: derivative on measurement, therefore
                                 minus sign in front of equation! */
        + (2.0f * tau - T) * differentiator) /
      (2.0f * tau + T);

  /*
   * Compute output and apply limits
   */
  out = proportional + integrator + differentiator;

  if (out > limMax) {

    out = limMax;

  } else if (out < limMin) {

    out = limMin;
  }

  /* Store error and measurement for later use */
  prevError = error;
  prevMeasurement = measurement;

  /* Return controller output */
  return out;
}
