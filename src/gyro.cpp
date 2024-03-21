#include "gyro.h"
#include "Arduino.h"

//define global variables
#define GYRO_PIN A3

float gyroSensitivity = 0.007;

int zeroGyroVoltage = 0;


void initiliseGyro(void){
    int sum = 0, sensorValue = 0, i=0; 
    
    for (i=0;i<100;i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
{
        sensorValue = analogRead(GYRO_PIN);
        sum += sensorValue;
        delay(5);
}
    zeroGyroVoltage  = sum/100;
}

float getAngularVelocity(void){
    int V, diff;
    float w;
    V = analogRead(GYRO_PIN);
    diff = V - zeroGyroVoltage;

    w = (float)diff/gyroSensitivity * 0.00488; //calculate angular velocity

    return w;
}

//it might make sense for this to be a class- then can have getAngle method and compute() in main loop
//get angle must be called in the main loop
float getAngle(long T){ 

    static long prev_millis;
    static float currentAngle = 0;
    long now = millis();
    float currentSpeed;

    if((prev_millis + T) < now ){ //only run logic at specified period
        currentSpeed = getAngularVelocity();
        currentAngle += currentSpeed*(float)T; // calculate the current angle
        return currentAngle;
    }
    //if 
    return -1.0;
    
    

}

void resetAngle(void){}

