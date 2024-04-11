
#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Servo.h>
#include <ir.h>
#include <PID_V2.h>

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

STATE wallFollow(float wallDist, float dist, IRSensorInterface* sensor, PID_v2* pidController);
STATE wallFollowRev(float wallDist, float dist, IRSensorInterface* sensor, IRSensorInterface* back, PID_v2* pidController);
STATE homing(IRSensorInterface* left, IRSensorInterface* right, IRSensorInterface* back );

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
STATE strafe_right(IRSensorInterface* sensor);

#endif

