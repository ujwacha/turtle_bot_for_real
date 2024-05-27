#include "stm32f4xx_hal.h"
#include <cstdint>
class PID {
public:
  /* Controller gains */
  float Kp;
  float Ki;
  float Kd;
  
  /* Derivative low-pass filter time constant */
  float tau;
  
  /* Output limits */
  float limMin;
  float limMax;
  
  /* Integrator limits */
  float limMinInt;
  float limMaxInt;
  
  /* Sample time (in seconds) */
  float T;
  
  /* Controller "memory" */
  float integrator = 0.0f;
  float prevError = 0.0f;			/* Required for integrator */
  float differentiator = 0.0f;
  float prevMeasurement = 0.0f;		/* Required for differentiator */
  uint32_t prev_time;
  /* Controller output */
  float out = 0.0f;
  
  PID(float _kp, float _ki, float _kd,
      float _T=0.030,
      float _tau = 0.01,
      float _min = -499.0f , float _max = 499.0f,
      float _intmin = -400.0f, float _intmax = +400.0f);


  void set_limits(float _min , float _max);
  void set_int_limits(float _intmin , float _intmax );
  void set_sampling_time(float _T);
  void set_tau(float _tau);

  float PIDController_Update(float setpoint, float measurement);

};
