#ifndef HOMING_H
#define HOMING_H
#include <ir.h>

enum initStates {
  SMALL_STRAFE_I,
  POLL,
  TURN_1, //turn to other side
  STRAFE_1, //straffe right to wall
  ULT, //if long -> SMALL STRAFE
  //long wall
  REV_1, //-> return FORWARD //not working // wall follow forward

  //short wall
  STRAFE_2, //straffe right
  TURN_2, // turn clockwise
  ULT_2, //check long or short
    //long
    STRAFE_3, //STRAFE_LEFT
    //SMALL_STRAFE_I -> REV 1
    //short
    TURN_3, //turn clockwise
    REV_2 //drive backwards
    
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