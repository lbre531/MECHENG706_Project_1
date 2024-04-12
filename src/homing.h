#ifndef HOMING_H
#define HOMING_H
#include <ir.h>

enum initStates {
  INIT,
  TURN_1,
  STRAFE_1,
  WALLCHECK,
  //long wall
  REV_I, //-> to small strafe
  //shortwall
  STRAFE_2,
  TURN_2,
  STRAFE_3, //-> back to wall check
  //corner
  STRAFE_4,
  TURN_3// back to wall check

} ;

enum wallPos{
  long_wall,
  short_wall,
  corner_wall
};

//include function definitions
initStates turnToWall(IRSensorInterface* left, float* angle);
// initStates straffeWall(IRSensorInterface* left);
wallPos wallCheck(IRSensorInterface* back);

// //found long wall
// initStates revToStart(IRSensorInterface *back);

// //found short wall

// //straffe out then turn 90degrees 
// initStates 

//in corner

#endif