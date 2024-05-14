// #include <arm_math.h>
#include "kinematics.hpp"

void kinematics::get_motor_omegas()
{
   float twist[3];
   twist[0] = Vx;
   twist[1] = Vy;
   twist[2] = omega * base_radius;

   for (int i = 0; i < 4; i++)
   {
      for (int j = 0; j < 3; j++)
      {
         motor_omegas[i] += inv_matrix[i][j] * twist[j];
      }
      motor_omegas[i] /= wheel_radius;
      v0 = motor_omegas[0];
      v1 = motor_omegas[1];
      v2 = motor_omegas[2];
      v3 = motor_omegas[3];
   }
}
//=====================================================================================
// to change after fixing makefile
float kinematics::arm_sin_f32(float degree)
{
   return degree;
}

float kinematics::arm_cos_f32(float degree)
{
   return 1.0f;
}
//=====================================================================================

void kinematics::set_value(float velocity, float theta_deg, float omega_inp)
{
   Vx = velocity * arm_sin_f32(theta_deg);
   Vy = velocity * arm_cos_f32(theta_deg);
   omega = omega_inp;
   get_motor_omegas();
}
