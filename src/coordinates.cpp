#include <Arduino.h> // Include Arduino core library if necessary
#include "coordinates.h"
#include "gyro.h"
#include "ir.h"
#include "movement.h"

// #include <SoftwareSerial.h>
// SoftwareSerial BluetoothSerial(10, 11);

bool initial1 = true, initial2 = true;
float x, y;
float xTable, yTable, angle;
float xOffset = 15, yOffset = 0; // add/subtract

float getCoordinates(long period, IRSensorInterface *Left, IRSensorInterface *Right)
{
    // BluetoothSerial.begin(115200);

    Left->readSensor(10);
    Right->readSensor(10);

    static long prev_millis;
    long current = millis();

    if (initial1)
    {
        xTable = HC_SR04_range();
        // yTable = LeftFront->getOutput();

        initial1 = false;
    }

    if ((prev_millis + period) < current)
    {
        // Copy values from PuTTy and use Excel file "CoordinatesMapped.xlsx"
        // BluetoothSerial.print(x);
        // BluetoothSerial.print(" ");
        // BluetoothSerial.println(y);

        x = xTable - HC_SR04_range() + xOffset; // ultrasonic
        y = -Left->getOutput() + yOffset;       // left IR

        if (y < -70)
        {
            if (initial2)
            {
                yTable = Right->getOutput();

                initial2 = false;
            }

            yOffset = -35;

            y = -(yTable - Right->getOutput()) + yOffset; // right IR

            yOffset = 0;
        }

        angle = getAngle(); // gyro

        if (abs(getAngle()) > 170) // if we turn 180 degrees midway
        {
            x = HC_SR04_range() + xOffset;            // ultrasonic
            y = Left->getOutput() - yTable + yOffset; // left IR
        }

        // update the time for the last loop
        prev_millis = current;
    }
}
