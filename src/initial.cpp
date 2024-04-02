#include "initial.h"
#include "Arduino.h"
#include "ir.h"
#include <movement.h>


// Set up the IR sensors
IRSensorInterface left_ir;
IRSensorInterface right_ir;
IRSensorInterface back_ir;

float left_dist;
float right_dist;
float back_dist;


void enable_ir () { // Attach ir sensors to pins
    left_ir.begin(A5);
    right_ir.begin(A6)
    back_ir.begin(A7);
}

void wall_Align ()
{
    left_dist = left_ir.getOutput();
    right_dist = right_ir.getOutput();

    // If left IR sensor is closer, rotate anti-clockwise
    // If right IR sensor is closer, rotate clockwise
    // Align when both IR sensor reading are approximately identical
    while (right_dist != left_dist) {
        if (right_dist > left_dist) {
            // rotate ccw
        } else {
            //rotate cw
        }
    }

}

void initial_Routine ()
{
    // Inititally reverse from wherever the robot starts
    void reverse ();

    // Somehow detect when we have collided with a wall
    float back_dist = back_ir.getOutput();
    if (back_dist <= 5) {  // Need to measure and adjust accordingly
        stop();
    }

    // Rotate robot until short range IR sensors on the side of the robot are detecting the wall 

    // Using wall_Align (to be written), align with wall
    wall_Align();

    // Reverse into a corner
    reverse();

    // If ultrasonic is measuring distance to opposite wall being > 1.8m, we are currently against a long
    // wall and are in the correct starting position

        // Begin routine
    
    // Else if the ultrasonic is measuring distance to opposite wall being < 1.5m, we are currently against a
    // short wall and 90 degrees out 

        // Move forward ever so slightly
        forward();
        // delay?
    
        // Move right ever so slightly
        // Rotate robot 90 degrees clockwise
        // Strafe left until robot comes to wall, using short IR for alignment again
        left_dist = left_ir.getOutput();
        right_dist = right_ir.getOutput();
        
        // Reverse back a little bit
        reverse();

        // Begin routine


}