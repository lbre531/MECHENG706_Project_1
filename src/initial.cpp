#include "initial.h"
#include "Arduino.h"
#include <movement.h>


void wall_Align ()
{

    // If left IR sensor is closer, rotate anti-clockwise
    // If right IR sensor is closer, rotate clockwise
    // Align when both IR sensor reading are approximately identical

}

void initial_Routine ()
{
    // Inititally reverse from wherever the robot starts
    void reverse ();

    // Somehow detect when we have collided with a wall

    // Rotate robot until short range IR sensors on the side of the robot are detecting the wall 

    // Using wall_Align (to be written), align with wall

    // Reverse into a corner

    // If ultrasonic is measuring distance to opposite wall being > 1.8m, we are currently against a long
    // wall and are in the correct starting position

        // Begin routine
    
    // Else if the ultrasonic is measuring distance to opposite wall being < 1.5m, we are currently against a
    // short wall and 90 degrees out 

        // Move forward ever so slightly
        // Move right ever so slightly
        // Rotate robot 90 degrees clockwise
        // Strafe left until robot comes to wall, using short IR for alignment again
        // Reverse back a little bit

        // Begin routine


}