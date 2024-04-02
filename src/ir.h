/*
How to use IRSensorInterface class:
Initilise IRSensorInterface instance and use begin() method to attach to a specific pin
Call readSensor(); repeatedly in the loop, it will only update the sensor measurement at the specified period
Use getOutput(); to read the current value of the sensor
*/





#ifndef IR_SENSOR_INTERFACE_H
#define IR_SENSOR_INTERFACE_H

#include <Arduino.h>  // Include Arduino core library if necessary

class IRSensorInterface {
private:
    int pin;
    float current_reading;
    
public:
    IRSensorInterface::IRSensorInterface();
    void IRSensorInterface::begin(int pin);
    bool IRSensorInterface::readSensor(long period);
    float IRSensorInterface::getOutput();

private:
    float IRSensorInterface::applyFilter(float rawValue);

};

#endif