#ifndef COORDINATES_H
#define COORDINATES_H


#include <Arduino.h> // Include Arduino core library if necessary
#include "ir.h"


float getCoordinates(long period, IRSensorInterface *LeftFront, IRSensorInterface *RightFront);


#endif
