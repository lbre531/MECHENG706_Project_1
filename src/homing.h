#ifndef HOMING_H
#define HOMING_H
#include <ir.h>

enum initStates {
  INIT,
  STRAFE_I,
  WALLCHECK,
  MOVE,
  TURN_I
} ;

//include function definitions
initStates homeInit(IRSensorInterface* left, IRSensorInterface* right);


#endif