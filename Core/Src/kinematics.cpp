#include "kinematics.hpp"
#include <arm_math.h>

void Kinematics::get_motor_omegas(float Vx,float Vy, float omega)
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

Kinematics::Kinematics(float _base_radius, float _wheel_radius)
{
  base_radius = _base_radius;
  wheel_radius = _wheel_radius;
  inv_matrix[0][0] = -0.3535533905932738;
  inv_matrix[0][1] = 0.3535533905932737;
  inv_matrix[0][2] = 0.25;
  
  inv_matrix[1][0] = 0.3535533905932738;
  inv_matrix[1][1] = 0.3535533905932738;
  inv_matrix[1][2] = -0.25;

  inv_matrix[2][0] = -0.3535533905932738;
  inv_matrix[2][1] = 0.3535533905932738;
  inv_matrix[2][2] = -0.25;
  
  inv_matrix[3][0] = 0.3535533905932738;
  inv_matrix[3][1] = 0.3535533905932738;
  inv_matrix[3][2] = 0.25;
}


