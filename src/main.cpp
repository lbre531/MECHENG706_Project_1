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
#include <SoftwareSerial.h>
#include <ir.h>
#include <PID_V2.h>
// #include <coordinates.h>


//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.



//Refer to Shield Pinouts.jpg for pin locations

//Bluetooth 
// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1
//volatile int32_t Counter = 1;

//SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);


//Default motor control pins


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.


//int speed_val = 100;
// int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;
//SerialCom = &BluetoothSerial;

//IR sensors
IRSensorInterface frontLeft;
IRSensorInterface backLeft;
IRSensorInterface backRight;
IRSensorInterface back;

//PID Controller
double Kp = 1, Ki = 0, Kd = 0;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);


//function definitions:
STATE initialising();
STATE running();
STATE stopped();
void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
boolean is_battery_voltage_OK();
void Analog_Range_A4();
void GYRO_reading();
// void read_serial_command();



int pos = 0;
void setup(void)
{
  //initilise PID controller 
  myPID.Start(50   ,  // input
              0,   // current output
              50);// setpoint
  myPID.SetOutputLimits(-100, 100);
  myPID.SetControllerDirection(PID::Direct);
  
  
  pinMode(LED_BUILTIN, OUTPUT);

  //include input for gyro pin

  // The Trigger pin will tell the sensor to range find
  initiliseUltrasonic();
  //setup IR sensors
  frontLeft.begin(A5,1844.3, -0.955);//shortRange
  backLeft.begin(A6,1,1); //longRange
  backRight.begin(A7,1,1); //longRange
  back.begin(A2,1,1); //shortRange


  //setup gyro
  initiliseGyro();


  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

 // BluetoothSerial.begin(115200);

  delay(1000); //settling time but no really needed

}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}



bool PID=0;
STATE running() {

  fast_flash_double_LED_builtin();
  
  // sensor1.readSensor(50); //poll IR sensor at 50ms period
  static STATE running_machine_state = HOME;

  #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
  #endif

  //Path Tracking...
  // getCoordinates(100, &backLeft, &backRight);

  //Finite-state machine Code
  switch (running_machine_state) {
    case HOME:
      running_machine_state = homing(&backLeft, &backRight, &back);
      break;
    case FORWARD: 
      running_machine_state = wallFollow(10 ,&frontLeft);
      break;
    case STRAFE:
      running_machine_state = strafe_right(); 
      break;
    case REV:
      running_machine_state = wallFollowRev(30, &backLeft);
      break;
    case TURN:
      running_machine_state = turnAngle(60);
      break;
      
  };

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  stop();
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}


#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

void Analog_Range_A4()
{
  // SerialCom->print("Analog Range A4:");
  // SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
//   SerialCom->print("GYRO A3:");
//   SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
// void read_serial_command()
// {
//   if (SerialCom->available()) {
//     char val = SerialCom->read();

//     //Perform an action depending on the command
//     switch (val) {
//       case 'w'://Move Forward
//       case 'W':
//         forward ();
//         SerialCom->println("Forward");
//         break;
//       case 's'://Move Backwards
//       case 'S':
//         reverse ();
//         SerialCom->println("Backwards");
//         break;
//       case 'q'://Turn Left
//       case 'Q':
//         strafe_left();
//         SerialCom->println("Strafe Left");
//         break;
//       case 'e'://Turn Right
//       case 'E':
//         strafe_right();
//         SerialCom->println("Strafe Right");
//         break;
//       case 'a'://Turn Right
//       case 'A':
//         ccw();
//         SerialCom->println("ccw");
//         break;
//       case 'd'://Turn Right
//       case 'D':
//         cw();
//         SerialCom->println("cw");
//         break;
//       case '-'://Turn Right
//       case '_':
//         speed_change = -100;
//         SerialCom->println("-100");
//         break;
//       case '=':
//       case '+':
//         speed_change = 100;
//         SerialCom->println("+");
//         break;
//       default:
//         stop();
//         SerialCom->println("stop");
//         break;
//     }

//   }

// }

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control
