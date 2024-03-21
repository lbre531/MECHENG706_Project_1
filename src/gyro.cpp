#include "gyro.h"
#include "Arduino.h"

//define global variables
#define GYRO_PIN A3

float gyroSensitivity = 0.007;
float T;

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

}