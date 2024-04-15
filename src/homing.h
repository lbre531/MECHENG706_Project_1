#ifndef HOMING_H
#define HOMING_H
#include <ir.h>

enum initStates {
  POLL,
  TURN_1, //turn to other side
  STRAFE_1, //straffe right to wall
  ULT,
  //long wall
  REV_1, //-> return FORWARD //not working // wall follow forward

  //short wall
  STRAFE_2, //straffe left
  TURN_2, // turn anti clockwise
  ULT_2,
    //long
    STRAFE_3, //straffe right to wall
    //short
    TURN_3, //turn clockwise
    REV_2 //drive forward function
    
  //assume in corner and strafe out and in

} ;

enum wallPos{
  long_wall,
  short_wall,
  corner_wall
};



//include function definitions
initStates poll(IRSensorInterface* left, IRSensorInterface* right, IRSensorInterface* back, float* angle);

initStates turnToWall(IRSensorInterface* left, float* angle);
// initStates straffeWall(IRSensorInterface* left);
wallPos wallCheck(IRSensorInterface* back);

initStates readUlt(IRSensorInterface* back);
initStates readUltOnly();
// //found long wall
// initStates revToStart(IRSensorInterface *back);

// //found short wall

// //straffe out then turn 90degrees 
// initStates 

//in corner

#endif