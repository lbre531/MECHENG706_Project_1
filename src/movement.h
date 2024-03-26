#include <Servo.h>

#ifndef MOVEMENT_H
#define MOVEMENT_H

//include function definitions
void forwardBias (int bias);
void strafe_right_bias( int bias);
void disable_motors();
void enable_motors();
void forward();
void reverse ();
void stop();
void ccw ();
void cw();
void strafe_left();
void strafe_right();

#endif

