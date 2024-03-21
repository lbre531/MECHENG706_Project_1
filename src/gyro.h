#ifndef GYRO_H
#define GYRO_H
//include function definitions
void initiliseGyro(void);
float getAngularVelocity(void);
void resetAngle(void);
bool calcAngle(long T);
float getAngle(void);


#endif