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



/*
Homing takes as inputs pointers to the relevant sensors required to be used, so they are available to be used*/
STATE homing(IRSensorInterface* left, IRSensorInterface* right, IRSensorInterface* back ){
  
  //return FORWARD;

  initStates init_states = INIT;
  //poll all sensors
  // left->readSensor(25);
  // right->readSensor(25);
  // back->readSensor(25);



  switch (init_states) {
    case HOME:
      init_states = turnToWall(left);
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
  float inches;

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
  inches = pulse_width / 148.0;
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

#include <SoftwareSerial.h>
//uncomment for testing
SoftwareSerial BluetoothSerial(10, 11);

STATE wallFollow(float wallDist, float dist, IRSensorInterface* sensor, PID_v2* pidController){
    // double Kp = 1, Ki = 0, Kd = 0;
    // static PID_v2 wallPID(Kp, Ki, Kd, PID::Reverse);
    
    //initilise variables
    static int counter = 0;
    static double output;
    double input, prevOutput;
    BluetoothSerial.begin(115200);
    
    static bool init = 1;

   if(init){
    pidController->SetTunings(5,0,0);//kp,ki, kd
    pidController->Setpoint(wallDist);
    pidController->SetOutputLimits(-100, 100);
    pidController->SetControllerDirection(PID::Reverse);

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
    BluetoothSerial.print(HC_SR04_range());

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
    BluetoothSerial.begin(115200);
    
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
      BluetoothSerial.print("input: ");
      BluetoothSerial.print(input);


      BluetoothSerial.print(", output: ");
      BluetoothSerial.println(output);
    
    //check if the robot is about to hit a wall
    if(back->getOutput()<dist){
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
STATE turnAngle(float angle){
  double Kp = 0.1, Ki = 0, Kd = 0;
  PID_v2 turnPID(Kp, Ki, Kd, PID::Reverse);
  
  float currentAngle = 0;
  static int counter = 0;
  static bool init = 1;

   if(init){
    // BluetoothSerial.begin(115200);
    // BluetoothSerial.print("init");
    resetAngle();//set gyro output to zero
    
    turnPID.Start(0   ,  // input
              1,   // current output
              angle);// setpoint

   turnPID.SetOutputLimits(-1, 1);

    init = 0;

   }
  
  long currentTime = millis();
  static long lastTime = currentTime;
  float output = 0;
 
      //calculate the currentAngle
      calcAngle(5);//calculate current angle with period of 5
      currentAngle = getAngle();
      
      //BluetoothSerial.println(currentAngle);
      output = turnPID.Run(currentAngle);

      //print output for debugging
      // BluetoothSerial.print(currentAngle);
      // BluetoothSerial.print(", ");
      currentTime = millis();
      
    if(lastTime + 100 < currentTime){
      lastTime = currentTime;

      // BluetoothSerial.print(output);
      // BluetoothSerial.print(", ");
      // BluetoothSerial.println(currentAngle);

      ccw(output);
    }

      // if(abs(currentAngle-angle) < 2){
      //     counter++;
      //     //BluetoothSerial.println(counter);
      //     if(counter > 5){
      //       init = 1;
      //       return STOPPED;
      //     } 
      // }

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

STATE strafe_right (IRSensorInterface* sensor)
{

  static long t0 = millis();

  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);

  if(t0 + 1000 < millis()) return REV; //reverse after time period of 1 second

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
