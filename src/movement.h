#include <Servo.h>
#include <ir.h>


#ifndef MOVEMENT_H
#define MOVEMENT_H


enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED,
  HOME,
  FORWARD,
  STRAFE,
  REV,
  TURN
};

//include function definitions
void initiliseUltrasonic(void);
float HC_SR04_range();

STATE wallFollow(float dist, IRSensorInterface* sensor);
STATE wallFollowRev(float dist, IRSensorInterface* sensor);
STATE homing();

STATE turnAngle(float angle);

void forwardBias (int bias);
void reverseBias (int bias);
void strafe_right_bias( int bias);
void disable_motors();
void enable_motors();
void forward();
void reverse ();
void stop();
void ccw (float speed);
void cw();
void strafe_left();
STATE strafe_right();

#endif

