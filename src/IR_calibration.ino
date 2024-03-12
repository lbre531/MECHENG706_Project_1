
int irsensor = A0;    //sensor is attached on pinA0
byte serialRead = 0;  //for control serial communication
int signalADC = 0;    // the read out signal in 0-1023 corresponding to 0-5V

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // start serial communication
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available())  // Check for input from terminal
  {
    serialRead = Serial.read();  // Read input
    if (serialRead == 49)        // Check for flag to execute, 49 is ascii for 1, stop serial printing
    {
      Serial.end();  // end the serial communication to get the sensor data
    }
  }

  signalADC = analogRead(irsensor);                // the read out is a signal from 0-1023 corresponding to 0-5v
  int distance1 = 17948 * pow(signalADC, -1.22);   // calculate the distance using the datasheet graph
  int distancec = 46161 * pow(signalADC, -1.302);  // calculate the distance using the calibrated graph
  Serial.print("distance datasheet graph ");       //print out the results on serial monitor
  Serial.print(distance1);
  Serial.println("cm");
  Serial.print(" ");
  Serial.print("distance calibration graph ");
  Serial.print(distancec);
  Serial.println("cm");
}
