#ifndef ROBOT_H_
#define ROBOT_H_

#include "stm32f4xx.h"

#ifdef __cplusplus

class Robot {
public:
  Robot() {}
  void init() {}
  void run() {}
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void init_robot();
void operate_robot();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ROBOT_H_
