#include <movement.h>
#include <Servo.h>
#include <Arduino.h>
#include <PID_V2.h>
#include <gyro.h>
#include <ir.h>
#include <homing.h>

const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29


int speed_val = 150;
//include the code for functions

const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

const unsigned int MAX_DIST = 23200;

void initiliseUltrasonic(void){
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

// #include <SoftwareSerial.h>
// //uncomment for testing
// SoftwareSerial BluetoothSerial(10, 11);


STATE revGyro(PID_v2* pidController, IRSensorInterface* sensor){

  static bool init = 1;
  static double output;
  static int counter = 0;
  double prevOutput, input;

   if(init){
    resetAngle();
    pidController->SetTunings(20,0,0);//kp,ki, kd
    pidController->Setpoint(0);
    pidController->SetOutputLimits(-100, 100);
    pidController->SetControllerDirection(PID::Reverse);
      
    // BluetoothSerial.begin(115200);

    init = 0;
   }
  
      input = getAngle();
      
      prevOutput = output;
      output = pidController->Run(input);   
      
      // currentTime = millis();

    if(prevOutput!= output){//if controller runs //this doesn't account for saturation
      // lastTime =currentTime; //resent last time

      //update output
      reverseBias(output);
      // BluetoothSerial.print("input: ");
      // BluetoothSerial.print(input);


      // BluetoothSerial.print(", output: ");
      // BluetoothSerial.println(output);
      // BluetoothSerial.print("output: ");
      // BluetoothSerial.println(sensor->getOutput());

    //check if the robot is about to hit a wall
    if(sensor->poll_return()<10){
      counter++;
      if(counter > 3){
        init = 1;
        return STRAFE;
      }
    }else{
        counter = 0;
    }
    }
    return REV;
}



STATE forwardGyro(PID_v2* pidController){

  static bool init = 1;
  static double output;
  static int counter = 0;
  double prevOutput, input;

   if(init){
    resetAngle();
    pidController->SetTunings(20,0,0);//kp,ki, kd
    pidController->Setpoint(0);
    pidController->SetOutputLimits(-100, 100);
    pidController->SetControllerDirection(PID::Reverse);
      
    // BluetoothSerial.begin(115200);

    init = 0;
   }
  
      input = getAngle();
      
      prevOutput = output;
      output = pidController->Run(input);   
      
      // currentTime = millis();

    if(prevOutput!= output){//if controller runs
      // lastTime =currentTime; //resent last time

      //update output
      forwardBias(output);
      // BluetoothSerial.print("input: ");
      // BluetoothSerial.print(input);


      // BluetoothSerial.print(", output: ");
      // BluetoothSerial.println(output);
    
    //check if the robot is about to hit a wall
    if(HC_SR04_range()<10){
      counter++;
      if(counter > 3){
        init = 1;
        return STRAFE;
      }
    }else{
        counter = 0;

    }
    }
    return FORWARD;
}





/*
Homing takes as inputs pointers to the relevant sensors required to be used, so they are available to be used*/
STATE homing(IRSensorInterface* left, IRSensorInterface* right, IRSensorInterface* back, PID_v2* pidController ){
  
  //return FORWARD;
  static bool init = 1;

  if (init){
    //  BluetoothSerial.begin(115200);
    init = 0;
  }
  
 
  static initStates init_states = POLL;
  //poll all sensors
  // left->readSensor(25);
  // right->readSensor(25);
  // back->readSensor(25);

  static float angle;

  switch (init_states) {
    case POLL:
        init_states = poll(left,right,back, &angle);
        // BluetoothSerial.print(angle);
    break;
    case TURN_1:
        
        if(turnAngle(angle, pidController) != STATE::TURN){
          init_states = STRAFE_1;
        }else{
          init_states = TURN_1;
        }
    break;
    case STRAFE_1:
        init_states= strafe_left_wall(left);
    break;
    case ULT:
      
      init_states = readUlt(back);
    break;
    case REV_1: //long walls
      if(wallFollowRev(10, 10, left, back, pidController) != STATE::REV){
        return FORWARD;
      }else{
        init_states = REV_1;
      }
    break;
    case STRAFE_2: //strafe right for 500ms
      if(strafe_right(right, 500) != STATE::STRAFE){
        init_states = TURN_2;
      }else{
        init_states = STRAFE_2;
      }
        //turn
    break;
    case TURN_2:
        if(turnAngle(90, pidController) != STATE::TURN){
          init_states = ULT_2;
        }else{
          init_states = TURN_2;
        }
    break;
    case ULT_2:
        init_states = readUltOnly();
    break;
      case STRAFE_3://the ult reads long
        if(strafe_left_wall(left) != STRAFE_1){
          return RUNNING;
        }
      break;
      case TURN_3://the ult reads short
        if(turnAngle(-90, pidController) != STATE::TURN){
          init_states = REV_2;
        }
      break;
      case REV_2:
        if(wallFollowRev(10, 10, left, back, pidController) != STATE::REV){
        return FORWARD;
        }
      break;
      

} 
  return HOME;

}


float HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      //the sensor is out of range
      return -1;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      //sensor out of range
      return -1;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  /*
  //apply filter
    static float previous_output;
    float sVal = 0.3; //define the smothing value
        //apply exponential filter possibly
        //y(k) = a * y(k-1) +  (1-a) * x(k)
        //tau = -T/log(a) 
    previous_output = sVal * previous_output + (1.0-sVal) * cm;
    
    previous_output = cm;
  */

  return cm;
}



STATE wallFollow(float wallDist, float dist, IRSensorInterface* sensor, PID_v2* pidController){
    // double Kp = 1, Ki = 0, Kd = 0;
    // static PID_v2 wallPID(Kp, Ki, Kd, PID::Reverse);
    
    //initilise variables
    static int counter = 0;
    static double output;
    double input, prevOutput;
    // BluetoothSerial.begin(115200);
    
    static bool init = 1;

   if(init){
    pidController->SetTunings(10,0,1);//kp,ki, kd
    pidController->Setpoint(wallDist);
    pidController->SetOutputLimits(-100, 100);
    pidController->SetControllerDirection(PID::Direct);

    init = 0;
   }
    

      sensor->readSensor(10); //read sensor with period of 10ms
      
      input = sensor->getOutput();
      
      prevOutput = output;
      output = pidController->Run(input);   
      
      // currentTime = millis();

    if(prevOutput!= output){//if controller runs
      // lastTime =currentTime; //resent last time

      //update output
      forwardBias(output);
      // BluetoothSerial.print("input: ");
      // BluetoothSerial.print(input);


      // BluetoothSerial.print(", output: ");
      // BluetoothSerial.println(output);
    
    //check if the robot is about to hit a wall
    // BluetoothSerial.print(HC_SR04_range());

    if(HC_SR04_range()<dist){
      counter++;
      if(counter > 3){
        init = 1;
        return STRAFE;
      }
    }else{
        counter = 0;

    }
    return FORWARD;
    }   
}

STATE wallFollowRev(float wallDist, float dist, IRSensorInterface* sensor, IRSensorInterface* back, PID_v2* pidController){
        // double Kp = 1, Ki = 0, Kd = 0;
    // static PID_v2 wallPID(Kp, Ki, Kd, PID::Reverse);
    
    //initilise variables
    static int counter = 0;
    static double output;
    double input, prevOutput;
    // BluetoothSerial.begin(115200);
    
    static bool init = 1;

   if(init){
    pidController->SetTunings(5,0,0);//kp,ki, kd
    pidController->Setpoint(wallDist);
    pidController->SetOutputLimits(-100, 100);
    pidController->SetControllerDirection(PID::Direct);

    init = 0;
   }
    

      // sensor->readSensor(10); //read sensor with period of 10ms
      // back->readSensor(10);
      
      input = sensor->getOutput();
      
      prevOutput = output;
      output = pidController->Run(input);   
      
      // currentTime = millis();

    if(prevOutput!= output){//if controller runs
      // lastTime =currentTime; //resent last time

      //update output
      reverseBias(output);
      // BluetoothSerial.print("input: ");
      // BluetoothSerial.print(input);


      // BluetoothSerial.print(", output: ");
      // BluetoothSerial.println(output);
    
    //check if the robot is about to hit a wall
    if(back->poll_return()<dist){
      counter++;
      if(counter > 3){
        init = 1;
        return STRAFE;
      }
    }else{
        counter = 0;

    }
    return REV;
    } 
}

//a positive angle is counter clockwise
STATE turnAngle(float angle, PID_v2* pidController){
  // double Kp = 0.1, Ki = 0, Kd = 0;
  // PID_v2 turnPID(Kp, Ki, Kd, PID::Reverse);
  
  float currentAngle = 0;
  static int counter = 0;
  static bool init = 1;

  

   if(init){
    // BluetoothSerial.begin(115200);
    // BluetoothSerial.print("init");
    resetAngle();//set gyro output to zero
    pidController->SetTunings(0.07,0.01,0.01);//kp,ki, kd
    pidController->Setpoint(angle);
    pidController->SetOutputLimits(-1, 1);
    pidController->SetControllerDirection(PID::Reverse);

    // BluetoothSerial.begin(115200);

    init = 0;

   }

    double input, prevOutput;
    static double output;
  
      input = getAngle();
      
      prevOutput = output;
      output = pidController->Run(input);   
      
      // currentTime = millis();

     if(prevOutput!= output){//if controller runs

      ccw(output);
      // BluetoothSerial.println(angle - input);


    }


      if(abs(input-angle) < 2){    
            init = 1;
            // BluetoothSerial.println("end");
            return STOPPED;
      }

      return TURN;
}



void forwardBias (int bias) //sensor reading
{
    
    left_font_motor.writeMicroseconds(1500 + speed_val - bias);
    left_rear_motor.writeMicroseconds(1500 + speed_val - bias);
    right_rear_motor.writeMicroseconds(1500 - speed_val - bias);
    right_font_motor.writeMicroseconds(1500 - speed_val - bias);
}



void reverseBias (int bias) //sensor reading
{
    
    left_font_motor.writeMicroseconds(1500 - speed_val - bias);
    left_rear_motor.writeMicroseconds(1500 - speed_val - bias);
    right_rear_motor.writeMicroseconds(1500 + speed_val -bias);
    right_font_motor.writeMicroseconds(1500 + speed_val -bias);
}

void strafe_right_bias( int bias){ //follow particular angle
  left_font_motor.writeMicroseconds(1500 + 100);
  left_rear_motor.writeMicroseconds(1500 - 100);
  right_rear_motor.writeMicroseconds(1500 - 92);
  right_font_motor.writeMicroseconds(1500 + 92);

}

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + 200);
  left_rear_motor.writeMicroseconds(1500 + 200);
  right_rear_motor.writeMicroseconds(1500 - 184);
  right_font_motor.writeMicroseconds(1500 - 184);
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw (float speed)
{
  left_font_motor.writeMicroseconds(1500 - speed*speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed*speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed*speed_val);
  right_font_motor.writeMicroseconds(1500 - speed*speed_val);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}
initStates strafe_left_wall(IRSensorInterface* sensor)
{
  
  static bool init = 0;

  static long startTime = millis();

  

  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);

  if(sensor->poll_return()<8) { 
    // BluetoothSerial.println(sensor->poll_return());
    if(!init){
      //  BluetoothSerial.println("jengjernnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnwingjwrengjerwngjrengjengorewngojrewngwo");
      startTime = millis();
      init = 1;
    }    
  }

  if (startTime + 1000 < millis() && init){
      // BluetoothSerial.println("end");
      init = 0;
      return ULT;
  }

  return STRAFE_1;
}

STATE strafe_right (IRSensorInterface* sensor, long time)
{

  static long t0;
  static bool init = 1;

  

   if(init){
    t0 = millis();
    init = 0;
   }

  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);

  if(t0 + time < millis()){
    init = 1;//set up function to be called again
    return REV;
  }  //reverse after time period of 1 second

  return STRAFE;




  // sensor->readSensor(10);
  
  // static int counter = 0;

  // static float initialDistance = sensor->getOutput();
  // float currentOutput;
  
  
  // static bool init = 1;


  // currentOutput = sensor-> getOutput(); 
  //  if(init){
  //   initialDistance = currentOutput;

  //   init = 0;
  //  }

  // left_font_motor.writeMicroseconds(1500 + speed_val);
  // left_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_font_motor.writeMicroseconds(1500 + speed_val);



  // BluetoothSerial.print(initialDistance);
  // BluetoothSerial.print(" ,");
  // BluetoothSerial.println(currentOutput);
  


  // if(abs(currentOutput - initialDistance) > 15 ){
  //   counter ++;
  //   if(counter >30){
  //       init = 1;
  //       return REV;
  //   }
  // }else{
  //   counter = 0;
  // }

  // return STRAFE;


}
