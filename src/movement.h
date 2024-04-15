
#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Servo.h>
#include <ir.h>
#include <PID_V2.h>
#include <homing.h>

enum STATE {
  FORWARD_WALL,
  BACK_WALL,
  
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

STATE wallFollow(float wallDist, float dist, IRSensorInterface* sensor, PID_v2* pidController, double kp, double ki, double kd);
STATE wallFollowRev(float wallDist, float dist, IRSensorInterface* sensor, IRSensorInterface* back, PID_v2* pidController, double kp, double ki, double kd);
STATE homing(IRSensorInterface* left, IRSensorInterface* right, IRSensorInterface* back, PID_v2* pidController  );

STATE forwardGyro(PID_v2* pidController);
STATE revGyro(PID_v2* pidController, IRSensorInterface* sensor);

STATE turnAngle(float angle, PID_v2* pidController);

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
STATE strafe_right(IRSensorInterface* sensor, long time);
initStates strafe_left_wall(IRSensorInterface* sensor);

#endif

