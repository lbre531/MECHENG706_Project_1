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
initStates turnToWall(IRSensorInterface* left);
// initStates straffeWall(IRSensorInterface* left);
// initStates isLong(IRSensorInterface*back);

// //found long wall
// initStates revToStart(IRSensorInterface *back);

// //found short wall

// //straffe out then turn 90degrees 
// initStates 

//in corner

#endif