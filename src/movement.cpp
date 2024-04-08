#include <movement.h>
#include <Servo.h>
#include <Arduino.h>
#include <PID_V2.h>
#include <gyro.h>
#include <ir.h>

const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

int speed_val = 100;
//include the code for functions

const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

const unsigned int MAX_DIST = 23200;

void initiliseUltrasonic(void){
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
}

STATE homing(){
  return FORWARD;

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

STATE wallFollow(float dist, IRSensorInterface* sensor){
    double Kp = 1, Ki = 0, Kd = 0;
    static PID_v2 wallPID(Kp, Ki, Kd, PID::Reverse);
    
    //initilise variables
    long currentTime = millis();
    static long lastTime;
    static int counter = 0;
    double output, input;
    BluetoothSerial.begin(115200);
    
    static bool init = 1;

   if(init){
    wallPID.Start(200   ,  // input
              0,   // current output
              200);// setpoint

    wallPID.SetOutputLimits(-100, 100);

    init = 0;
   }
    

      sensor->readSensor(10); //read sensor with period of 10ms
      input = sensor->getOutput();

      output = wallPID.Run(input);   
      
      currentTime = millis();
    if((lastTime + 25)  < currentTime){
      lastTime =currentTime; //resent last time

      //update output
      forwardBias(output);
      
      // BluetoothSerial.print("output: ");
      // BluetoothSerial.println(output);
    
    //check if the robot is about to hit a wall
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

STATE wallFollowRev(float dist, IRSensorInterface* sensor){
    double Kp = 1, Ki = 0, Kd = 0;
    static PID_v2 wallPID(Kp, Ki, Kd, PID::Reverse);
    
    //initilise variables
    long currentTime = millis();
    static long lastTime;
    static int counter = 0;
    double output, input;
    //BluetoothSerial.begin(115200);
    
    wallPID.Start(200   ,  // input
              0,   // current output
              200);// setpoint

    wallPID.SetOutputLimits(-100, 100);

    
    //poll sensor in every loop
    sensor->readSensor(10); //read sensor with period of 10ms
    input = sensor->getOutput();

      // BluetoothSerial.print("input: ");
      // BluetoothSerial.println(input);

    output = wallPID.Run(input);
    currentTime = millis();

      if((lastTime + 25)  < currentTime){ //run at period of 25ms
      lastTime =currentTime; //resent last time

      //update output
      reverseBias(1); //update the output
      
      // BluetoothSerial.print(" output: ");
      // BluetoothSerial.println(output);

    //check if the robot is about to hit a wall and change states if so
    if(HC_SR04_range()>(180-dist)){
      counter++;
      if(counter > 3) return STRAFE;
    }else {
      counter = 0;
      return FORWARD;
    }
    }
    
}


//a positive angle is counter clockwise
void turnAngle(float angle){
  double Kp = 10, Ki = 0, Kd = 0;
  PID_v2 turnPID(Kp, Ki, Kd, PID::Reverse);
  
  resetAngle();//set gyro output to zero
  float currentAngle = 0;
  int counter = 0;
  
  turnPID.Start(0   ,  // input
              0,   // current output
              angle);// setpoint

  turnPID.SetOutputLimits(-100, 100);
  
  float currentTime = millis();
  float lastTime = currentTime;
  float output = 0;

  bool done = 0;
  while(!done){
      //calculate the currentAngle
      currentAngle = getAngle();
      output = turnPID.Run(currentAngle);

      currentTime = millis();
      
    if(lastTime + 25 < currentTime){
      lastTime = currentTime;

      ccw(output);

    }
      if(abs(currentAngle-angle) < 2){
          counter++;
          if(counter > 3) done = 1;
      }else{
        counter = 0;
        done = 0;
      }
  }
}



void forwardBias (int bias) //sensor reading
{
    
    left_font_motor.writeMicroseconds(1500 + speed_val + bias);
    left_rear_motor.writeMicroseconds(1500 + speed_val + bias);
    right_rear_motor.writeMicroseconds(1490 - speed_val);
    right_font_motor.writeMicroseconds(1490 - speed_val);
}



void reverseBias (int bias) //sensor reading
{
    
    left_font_motor.writeMicroseconds(1435 - speed_val - bias);
    left_rear_motor.writeMicroseconds(1435 - speed_val - bias);
    right_rear_motor.writeMicroseconds(1525 + speed_val);
    right_font_motor.writeMicroseconds(1525 + speed_val);
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
  turret_motor.attach(11);
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

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
