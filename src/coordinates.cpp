#include <Arduino.h> // Include Arduino core library if necessary
#include "coordinates.h"
#include "gyro.h"
#include "ir.h"
#include "movement.h"


#include <SoftwareSerial.h>
SoftwareSerial BluetoothSerial(10, 11);


bool initial1 = true, initial2 = true;
float x, y, xPrev, yPrev, xTable, yTable, angle;
float xOffset = 15, yOffset = 0; // Add/subtract to adjust


float getCoordinates(long period, IRSensorInterface *Left, IRSensorInterface *Right)
{
    BluetoothSerial.begin(115200);
    Left->readSensor(10);
    Right->readSensor(10);
    static long prev_millis;
    long current = millis();


    // Record reference table length
    if (initial1)
    {
        xTable = HC_SR04_range(); // ultrasonic
        initial1 = false;
    }


    // Start recording previous values after initial x and y
    if (!initial1)
    {
        xPrev = x;
        yPrev = y;
    }


    if ((prev_millis + period) < current)
    {
        x = xTable - HC_SR04_range() + xOffset; // ultrasonic
        y = -Left->getOutput() + yOffset;       // left IR



        // If range of left IR is exceeded, switch to right IR
        if (y < -70)
        {
            // Record reference table length again
            if (initial2)
            {
                yTable = Right->getOutput();
                initial2 = false;
            }


            // Offset required for calculation
            yOffset = -75;
            y = -(yTable - Right->getOutput()) + yOffset; // right IR
            yOffset = 0;
        }

          // Makes sure values printed in PuTTy are not wild outliers
        if ((abs(x - xPrev) < 30) && (abs(y - yPrev) < 30))
        {
            // Copy values from PuTTy and use Excel file "CoordinatesMapped.xlsx"
            BluetoothSerial.print(x);
            BluetoothSerial.print(" ");
            BluetoothSerial.println(y);
        }


        // angle = getAngle(); // gyro


        // if (abs(getAngle()) > 170) // if we turn 180 degrees midway
        // {
        //     x = HC_SR04_range() + xOffset;            // ultrasonic
        //     y = Left->getOutput() - yTable + yOffset; // left IR
        // }


        // Update the time for the last loop
        prev_millis = current;
    }
}