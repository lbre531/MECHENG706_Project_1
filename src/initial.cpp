#include "initial.h"
#include "Arduino.h"
#include "ir.h"
#include "movement.h"


// Set up the IR sensors
IRSensorInterface left_ir;
IRSensorInterface right_front_ir;
IRSensorInterface right_back_ir;
IRSensorInterface back_ir;

float left_dist;
float right_front_dist;
float right_back_dist;
float back_dist;
float front_dist;


void enable_ir () { // Attach ir sensors to pins
    left_ir.begin(A5);
    right_front_ir.begin(A6);
    right_back_ir.begin(A7);
    back_ir.begin(A8);
}

void wall_Align ()
{
    left_dist = left_ir.getOutput();
    float right1 = right_front_ir.getOutput();
    float right2 = right_back_ir.getOutput();
    right_front_dist = (right1 + right2) / 2;

    // If left IR sensor is closer, rotate anti-clockwise
    // If right IR sensor is closer, rotate clockwise
    // Align when both IR sensor reading are approximately identical
    while (right_front_dist != left_dist) {
        if (right_front_dist > left_dist) {
            ccw();
        } else {
            cw();
        }
    }

}

void initial_Routine ()
{
    // Inititally reverse from wherever the robot starts
    // Somehow detect when we have collided with a wall
    back_dist = back_ir.getOutput();
    while(back_dist <= 50) { // in mm, Need to measure and adjust accordingly
        reverse();
        back_dist = back_ir.getOutput();
    }
    stop();

    // Rotate robot until short range IR sensors on the side of the robot are detecting the wall 
    // Using wall_Align (to be written), align with wall
    wall_Align();

    // Reverse into a corner
    back_dist = back_ir.getOutput();
    while(back_dist > 100) { // in mm, Need to measure and adjust accordingly
        reverse();
        back_dist = back_ir.getOutput();
    }
    stop();

    // If ultrasonic is measuring distance to opposite wall being > 1.8m, we are currently against a long
    // wall and are in the correct starting position

        // Begin routine
    
    // Else if the ultrasonic is measuring distance to opposite wall being < 1.5m, we are currently against a
    // short wall and 90 degrees out 

        // Move forward ever so slightly
        forward();
        // delay?
    
        // Move right ever so slightly
        strafe_right();

        // Rotate robot 90 degrees clockwise
        right_front_dist = right_front_ir.getOutput();
        right_back_dist = right_back_ir.getOutput();
        while(right_front_dist > right_back_dist) {
            cw();
            right_front_dist = right_front_ir.getOutput();
            right_back_dist = right_back_ir.getOutput();
        }
        stop();

        // Strafe left until robot comes to wall, using short IR for alignment again
        strafe_left();
        delay(500);
        stop();
        
        // Reverse back a little bit
        reverse();
        delay(1000);
        stop();

        // Begin routine


}