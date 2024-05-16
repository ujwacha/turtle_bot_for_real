#pragma once
class Kinematics{
 private:
  float motor_omegas[4]; // array to store motor omega
  double inv_matrix[4][3]; // inverse kinematics matrix
  float wheel_radius;
  float base_radius;
  float Vx,Vy,omega; // velocity and omega of base
					 

 public:
  float v1,v2,v3,v4; // these are motor omegas though they are written as v 
  void get_motor_omegas(float Vx, float Vy, float omega);
  Kinematics(float _base_radius, float _wheel_radius);
  void reset();
};
