#include <Arduino.h>  // Include Arduino core library if necessary
#include "ir.h"
#include <SoftwareSerial.h>
#include <movement.h>

SoftwareSerial BluetoothSerial(10,11);

    IRSensorInterface::IRSensorInterface(): pin(0), current_reading(0){} 
    
    void IRSensorInterface::begin(int pin){
        this->pin = pin;
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

            Serial.println(sensor_reading);
            // Serial.print(", ");
            // Serial.println(current_reading);
        }
    }
    
    float IRSensorInterface::getOutput(){
        return current_reading;
    }

    float IRSensorInterface::applyFilter(float rawValue){      
        static float previous_output;
        //apply exponential filter possibly
        //y(k) = a * y(k-1) +  (1-a) * x(k)
        //tau = -T/log(a) 
        previous_output = 0.2 * previous_output + 0.8 * rawValue; //check integer division
        
        return rawValue;
    }

    /*
    Method to calibrate sensor, returns sensor readings from sonar while ignoring large values
    */
    void IRSensorInterface::calibrate(){
    BluetoothSerial.begin(115200);

    static float prevSonar , sonar, ir;

    this->readSensor(10);

    sonar = HC_SR04_range();

    if((prevSonar + 2.5 < sonar) && (prevSonar + 5 < sonar)){
        prevSonar = sonar;
        
        ir = this->getOutput();

        BluetoothSerial.print(sonar);
        BluetoothSerial.print(", ");
        BluetoothSerial.println(ir);    
    }
    }