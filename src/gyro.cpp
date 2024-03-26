#include "gyro.h"
#include "Arduino.h"

//define global variables
#define GYRO_PIN A3

float gyroSensitivity = 0.0066;

int zeroGyroVoltage = 0;
float currentAngle = 0;

void initiliseGyro(void){
    int sum = 0, sensorValue = 0, i=0; 
    
    for (i=0;i<10;i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
{
        sensorValue = analogRead(GYRO_PIN);
        Serial.println(sensorValue);
        sum += sensorValue;
        delay(5);
        }

    Serial.print("sum is: ");
    Serial.println(sum);
    zeroGyroVoltage  = sum/10;
    
    Serial.print("zeroGyroVoltage is: ");
    Serial.println(zeroGyroVoltage);


    //for testing
    zeroGyroVoltage = 514;

    //why is this not printing?
    //Serial.print('zeroGyroVoltage is: ');
    //Serial.println(zeroGyroVoltage);
}

float getAngularVelocity(void){
    int V, diff;
    float w;
    V = analogRead(GYRO_PIN);
    diff = V - zeroGyroVoltage;

    w = (float)diff/gyroSensitivity * 0.00488; //calculate angular velocity

    return w;
}


//calculates angle- returns ture if the code runs
bool calcAngle(long T){ 

    
    static long prev_millis;
    long now = millis();
    float currentSpeed;

    if((prev_millis + T) < now ){ //only run logic at specified period
        currentSpeed = getAngularVelocity();
        currentAngle += currentSpeed*(float)T/1000.0; // calculate the current angle
        prev_millis = millis();
         Serial.print("currentSpeed");
         Serial.println(currentSpeed);
        Serial.print("currentAngle: ");
        Serial.println(currentAngle);
        return 1;
    }
   

    return 0;
}

float getAngle(){
    return currentAngle;
}


void resetAngle(void){
    currentAngle = 0;
}

