#pragma once
class Kinematics{
 private:
  float motor_omegas[4]; // array to store motor omega
  double inv_matrix[4][3]; // inverse kinematics matrix
  float wheel_radius;
  float base_radius;
  float Vx,Vy,omega; // velocity and omega of base
					 
  void get_motor_omegas();

 public:
  float v0,v1,v2,v3; // these are motor omegas though they are written as v 

  Kinematics(float _base_radius, float _wheel_radius);

  void set_value(float x,float y, float omega_inp);
};
