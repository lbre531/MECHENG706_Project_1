#include <Servo.h>
#include <ir.h>

#ifndef MOVEMENT_H
#define MOVEMENT_H

//include function definitions
void initiliseUltrasonic(void);
float HC_SR04_range();

void wallFollow(float dist, IRSensorInterface* sensor);
void wallFollowRev(float dist, IRSensorInterface* sensor);

void turnAngle(float angle);

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
void strafe_right();

#endif

