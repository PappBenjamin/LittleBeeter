#include <stdio.h>
#include <QTRSensors.h>
#include <Arduino.h>


/*DISCLAIMER*/
// For Motor Driver L we use these colors: GND Black, EN Brown, PWM Red
// For Motor Driver L we use these colors: GND White, EN Purple, PWM Blue



/*Pin Definition*/
int EN_left = 3;     // Enable Pin for Motor Driver L Connected to Digital 3
int EN_right = 2;     // Enable Pin for Motor Driver R Connected to Digital 2

int PWM_left = 26; // PWM Pin for Motor Driver L Connected to Digital 26
int PWM_right = 25; // PWM Pin for Motor Driver R Connected to Digital 25

int IRPins[] = {21};

const u_int8_t QTRPins[] = {18}; // QTR Sensor Pin Connected to Digital 18
QTRSensors qtr; // QTR Sensor Object
int SensorCount = 1;

int startPin = 19; // IR Pin Connected to Digital 21





void setup()
{
  //setting up motor driver pins
  pinMode(EN_left, OUTPUT);   // Enable Pin
  pinMode(EN_right, OUTPUT);  // Enable Pin

  digitalWrite(EN_left, LOW); // Enable Pin set low for avoding false startups.
  digitalWrite(EN_right, LOW); // Enable Pin set low for avoding false startups.

  /*-----------------------------------------------------------*/

  //setting up IR pins
  for (int i = 0; i < sizeof(IRPins) / sizeof(IRPins[0]); i++)
  {
    pinMode(IRPins[i], INPUT);
  }

  /*-----------------------------------------------------------*/

  //setting up QTR pins
  qtr.setTypeRC();
  qtr.setSensorPins(QTRPins, SensorCount);


  //calibrate QTR sensors
  for(int i = 0; i < 300; i++)
  {
    qtr.calibrate();
  }

  /*-----------------------------------------------------------*/
  //setting up start pin
  pinMode(startPin, INPUT);

  /*-----------------------------------------------------------*/



  delay(1500);           
  Serial.begin(9600);   
}

void testMotors(int EN, int PWM)
{
  Serial.println("FULL FORWARD");
  digitalWrite(EN, HIGH);
  analogWrite(PWM, 252); // Full FWD Speed (252)
  delay(2000);             // 2 Second Delay

  Serial.println("STOPPED");
  digitalWrite(EN, LOW);
  analogWrite(PWM, 128); // Half Duty Cycle and Enable Low Stops Motor (128)
  delay(1000);

  Serial.println("FULL BACKWARD");
  digitalWrite(EN, HIGH);
  analogWrite(PWM, 2); // Full BWD Speed (2)
  delay(2000);   

  Serial.println("STOPPED");
  digitalWrite(EN, LOW);
  analogWrite(PWM, 128); // Half Duty Cycle and Enable Low Stops Motor (128)
  delay(1000);
}



void loop()
{

  uint16_t sensorValules[SensorCount];
  qtr.readLineBlack(sensorValules); // Read the QTR sensor values

  int starter = digitalRead(startPin);

  if(starter)
  {
    Serial.print("Sensor Value: ");
    Serial.println(sensorValules[0]); // Print the sensor value
    delay(100); 
  }

  
}