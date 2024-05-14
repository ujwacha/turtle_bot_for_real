class kinematics{
 private:
  float motor_omegas[4]; // array to store motor omega
  double inv_matrix[4][3]; // inverse kinematics matrix
  float wheel_radius;
  float base_radius;
  float Vx,Vy,omega; // velocity and omega of base
					 
  void get_motor_omegas();

  //=====================================================================================
  // to change after fixing makefile
  float arm_sin_f32(float degree);
  float arm_cos_f32(float degree);
  //=====================================================================================

 public:
  float v0,v1,v2,v3; // these are motor omegas though they are written as v 

  kinematics(float _base_radius, float _wheel_radius)
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

  void set_value(float velocity,float theta_deg, float omega_inp);
};
