#include <homing.h>
#include <movement.h>
#include <gyro.h>

PID_v2 homePID(0,0,0,PID_v2::Reverse);

initStates turnToWall(IRSensorInterface* left, float* angle){  
    static float shortDistance = 1000;
    float result;
    static float bestAngle;
   
    static bool init=1;
    if(init){
    // BluetoothSerial.begin(115200);
    // BluetoothSerial.print("init");
    
    homePID.Start(0,0,0);
    init = 0;

   }
    left->readSensor(10);

    
        if(turnAngle(360, &homePID) != STATE::TURN){
            return TURN_1;
        }
        result = left->getOutput();

        if(result < shortDistance){
        shortDistance = result;
        *angle = getAngle();
        bestAngle = *angle;
        }else{
            *angle = bestAngle;
        }   

    return INIT;
}

//function to decide if the wall is long, short or corner

wallPos wallCheck(IRSensorInterface* back){
    back->readSensor(10);

    float length = back->getOutput() + HC_SR04_range();
    //50, 
    if (length < 30){
        return corner_wall;
    }
    if (length > 100){
        return long_wall;
    }else{
        return short_wall;
    }
}





