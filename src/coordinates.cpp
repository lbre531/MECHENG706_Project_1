#include <Arduino.h> // Include Arduino core library if necessary
#include "coordinates.h"
#include "gyro.h"
#include "ir.h"
#include "movement.h"


#include <SoftwareSerial.h>
SoftwareSerial BluetoothSerial(10, 11);


bool initial = true;


float x, y;
float xTable, yTable, angle;


float xOffset = 0, yOffset = 0; // add/subtract


float getCoordinates(long period, IRSensorInterface *LeftFront, IRSensorInterface *RightFront)
{
    BluetoothSerial.begin(115200);


    LeftFront->readSensor(10);
    RightFront->readSensor(10);


    static long prev_millis;
    long current = millis();


    if (initial)
    {
        xTable = HC_SR04_range();
        yTable = LeftFront->getOutput();


        initial = false;
    }


    if ((prev_millis + period) < current)
    {
        //BluetoothSerial.print("\t");
        BluetoothSerial.print(x);
        BluetoothSerial.print(" ");
        BluetoothSerial.println(y);


        // BluetoothSerial.print(HC_SR04_range());
        // BluetoothSerial.print("\t");
        // BluetoothSerial.print(LeftFront->getOutput());
        // BluetoothSerial.print("\t");
        // BluetoothSerial.println(getAngle());


        x = xTable - HC_SR04_range() + xOffset; // ultrasonic
        // BluetoothSerial.println("entered x");


        y = -LeftFront->getOutput() + yOffset; // left IR
        // BluetoothSerial.println("entered y");


        angle = getAngle(); // gyro


        if (abs(getAngle()) > 170)
        {
            x = HC_SR04_range() + xOffset; // ultrasonic
            // BluetoothSerial.println("entered x");


            y = LeftFront->getOutput() - yTable + yOffset; // left IR
            // BluetoothSerial.println("entered y");
        }


        // x = HC_SR04_range() * cos(angle) + xOffset;
        // //BluetoothSerial.println("entered xcos");
        // if (angle >= 0) { // angle is positive, CW
        //     y = -HC_SR04_range() * sin(angle) + yOffset;
        //     //BluetoothSerial.println("entered ysin1");
        // } else { // angle is negative, CCW
        //     y = HC_SR04_range() * sin(angle + yOffset);
        //     //BluetoothSerial.println("entered ysin2");
        // }


        // update the time for the last loop
        prev_millis = current;
    }
}
