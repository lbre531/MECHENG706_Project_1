/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/


//include all header files for other code here

#include <Servo.h>  //Need for Servo pulse output
#include <Arduino.h>
#include <movement.h>
#include <gyro.h>

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.




int speed_val = 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;



int pos = 0;
void setup(void)
{
  //initilise the gyro
  

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");
  
  
  initiliseGyro();
  
  delay(1000); //settling time but no really needed

}

  bool calcBool=0;
  float w;
void loop(void) //main loop
{
   calcBool = calcAngle(10);
   if (calcBool){
    w = getAngularVelocity();
    Serial.print("Angular velocity: ");
    Serial.println(w);
    
    Serial.print("Calculation period 100ms, angle is: ");
    Serial.println(getAngle());
   }

  enable_motors();


  speed_val = 100;


  left_font_motor.writeMicroseconds(1600);
  left_rear_motor.writeMicroseconds(1600);
  right_rear_motor.writeMicroseconds(1400);
  right_font_motor.writeMicroseconds(1500 - speed_val);


  if 0 < gyro_angle && gyro_angle < 90 {
    speed_val =+ 5;
  } else if 270 < gyro_angle && gyro_angle < 360 {
    speed_val =- 5;
  } else {
    stop()
    disable_motors();
  }



}
