#include <Arduino.h>  // Include Arduino core library if necessary

class IRSensorInterface {
private:
    int pin;
    float current_reading;

public:
    IRSensorInterface(int pin): pin(pin), current_reading(0){
        pinMode(pin, INPUT);
    }
    
    //method that reads the previos sensor value at a given period and applies digital filter
    //inputs : period is the time in milliseconds for each loop
    //outputs: the output is 1 if the sensor value is read and 0 otherwise

    bool readSensor(long period){
        static long prev_millis;
        long current = millis();
        
        int sensor_reading;

        if ((prev_millis + period) > current){
            sensor_reading = analogRead(pin);

            current_reading = applyFilter(sensor_reading);

            //update the time for the last loop
            prev_millis = current;
        }
    }
    
    int getOutput(){
        return current_reading;
    }

private:
    float applyFilter(int rawValue){
        //apply exponential filter possibly

        
    }
    
};