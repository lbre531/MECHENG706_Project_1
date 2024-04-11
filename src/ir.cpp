#include <Arduino.h>  // Include Arduino core library if necessary
#include "ir.h"

    IRSensorInterface::IRSensorInterface(): pin(0), current_reading(0){} 
    
    void IRSensorInterface::begin(int pin, float b1, float b2){
        this->pin = pin;
        this->b1 = b1;
        this->b2 = b2;
        pinMode(pin, INPUT);
    }
    
    //method that reads the previos sensor value at a given period and applies digital filter
    //inputs : period is the time in milliseconds for each loop
    //outputs: the output is 1 if the sensor value is read and 0 otherwise

    bool IRSensorInterface::readSensor(long period){
        static long prev_millis;
        long current = millis();
        
        float sensor_reading;

        if ((prev_millis + period) < current){
            sensor_reading = analogRead(pin);

            //update current reading
            current_reading = applyFilter(sensor_reading);

            //update the time for the last loop
            prev_millis = current;
        }
    }
    
    float IRSensorInterface::getOutput(){
        float result = b1*pow(current_reading,b2);
        if(result>80){
            return 80;
        }
        return result;  //change back for correct readings
    }

    float IRSensorInterface::applyFilter(float rawValue){      
        static float previous_output;
        //apply exponential filter possibly
        //y(k) = a * y(k-1) +  (1-a) * x(k)
        //tau = -T/log(a) 
        previous_output = 0.5 * previous_output + 0.5 * rawValue; //check integer division
        
        return rawValue;
    }