#ifndef IR_SENSOR_INTERFACE_H
#define IR_SENSOR_INTERFACE_H

#include <Arduino.h>  // Include Arduino core library if necessary

class IRSensorInterface {
private:
    int pin;
    float current_reading;

public:
    IRSensorInterface(int pin);
    bool readSensor(long period);
    int getOutput();

private:
    float applyFilter(float rawValue);
    
};

#endif