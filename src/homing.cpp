#include <homing.h>
#include <movement.h>
#include <gyro.h>

PID_v2 homePID(0,0,0,PID_v2::Reverse);


initStates readUltOnly(){
    
    if(HC_SR04_range()< 80){
        return TURN_3;
    }
    return STRAFE_3;
}


initStates readUlt(IRSensorInterface* back){

    float back_s;
    
    float ultraCoolSuperPowers = HC_SR04_range();
    
    back_s = back->poll_return();

    float result = ultraCoolSuperPowers + back_s;

    if (result > 100){
       return REV_1;
    }
    return STRAFE_2;

}


initStates poll(IRSensorInterface* left, IRSensorInterface* right, IRSensorInterface* back, float* angle){


    float left_s, right_s, back_s, front_s;

    left_s = left->poll_return();
    right_s = right->poll_return();
    back_s = back->poll_return();
    front_s = HC_SR04_range();

    float smallest = left_s;
    *angle = 0; // return strafe to wall

    if (right_s < smallest){
        smallest = right_s;
        *angle = 180;
    }
    if (back_s < smallest){
        smallest = back_s;
        *angle = -90;

    }
    if(front_s < smallest){
        smallest = front_s;
        *angle = 90;
    }

    if(*angle == 0){
        return STRAFE_1;
    }
    
    return TURN_1;
    
}

// initStates turnToWall(IRSensorInterface* left, float* angle){  
//     static float shortDistance = 1000;
//     float result;
//     static float bestAngle;
   
//     static bool init=1;
//     if(init){
//     // BluetoothSerial.begin(115200);
//     // BluetoothSerial.print("init");
    
//     homePID.Start(0,0,0);
//     init = 0;

//    }
//     left->readSensor(10);

    
//         if(turnAngle(360, &homePID) != STATE::TURN){
//             return TURN_1;
//         }
//         result = left->getOutput();

//         if(result < shortDistance){
//         shortDistance = result;
//         *angle = getAngle();
//         bestAngle = *angle;
//         }else{
//             *angle = bestAngle;
//         }   

//     return INIT;
// }

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





