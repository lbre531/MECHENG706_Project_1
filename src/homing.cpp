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






