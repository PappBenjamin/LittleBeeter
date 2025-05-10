#include <stdio.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include "hardware/pwm.h"
#include "math.h"

/*DISCLAIMER*/
// For Motor Driver L we use these colors: GND Black, EN Brown, PWM Red
// For Motor Driver R we use these colors: GND White, EN Purple, PWM Blue

/*Pin Definition*/
#define EN_left 8
#define EN_right 10

#define PWM_left 9
#define PWM_right 11

#define startPin 12 // starter pin

// Define IR pins for opponent detection
// sensor alignment: 13 back, 14 left, .. front left, .. center, .. front right, .. right
int IRPins[] = {13, 14};
int IRCount = 2;

// Define QTR sensor pins for edge detection
const uint8_t QTRPins[] = {2, 3, 4, 5};
QTRSensors qtr;
int SensorCount = 4;

// Define threshold values
#define EDGE_THRESHOLD 400   // Value that indicates ring edge
#define OPPONENT_THRESHOLD 1 // Value that indicates opponent detected

// Macros for motor speeds (0–255)
#define STOP_SPEED 128
#define ATTACK_SPEED_FWD 61
#define ATTACK_SPEED_BWD 195
#define RETREAT_SPEED_BWD 255
#define RETREAT_SPEED_FWD 0
#define SEARCH_SPEED_FWD 1
#define SEARCH_SPEED_BWD 215

#define PWM_FREQ 3900      // Target PWM frequency in Hz
#define PWM_RESOLUTION 255 // 8-bit resolution

// Store slice numbers
uint slice_left;
uint slice_right;

// Setup PWM frequency on a pin
void setupPWM(uint pin, uint &slice)
{
  gpio_set_function(pin, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num(pin);

  float divider = (float)125000000 / (PWM_FREQ * (PWM_RESOLUTION + 1));
  if (divider < 1.0f)
    divider = 1.0f;
  if (divider > 255.0f)
    divider = 255.0f;

  pwm_set_clkdiv(slice, divider);
  pwm_set_wrap(slice, PWM_RESOLUTION);
  pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0); // Initially 0
  pwm_set_enabled(slice, true);
}

void setup()
{
  // Motor driver enable pins
  pinMode(EN_left, OUTPUT);
  pinMode(EN_right, OUTPUT);

  digitalWrite(EN_left, LOW);
  digitalWrite(EN_right, LOW);

  // IR sensors
  for (int i = 0; i < IRCount; i++)
  {
    pinMode(IRPins[i], INPUT);
  }

  // QTR sensors
  qtr.setTypeRC();
  qtr.setSensorPins(QTRPins, SensorCount);

  for (int i = 0; i < 300; i++)
  {
    qtr.calibrate();
  }

  // Start pin
  pinMode(startPin, INPUT);

  // Set up PWM for both motors
  setupPWM(PWM_left, slice_left);
  setupPWM(PWM_right, slice_right);

  Serial.begin(9600);
  Serial.println("Sumo robot initialized. Waiting for start signal.");
}

// Function to set motor speed (0-255)
void setLeftMotor(int speed)
{
  digitalWrite(EN_left, HIGH);
  pwm_set_chan_level(slice_left, pwm_gpio_to_channel(PWM_left), speed);
}
void setRightMotor(int speed)
{
  digitalWrite(EN_right, HIGH);
  pwm_set_chan_level(slice_right, pwm_gpio_to_channel(PWM_right), speed);
}
void stopMotors()
{
  digitalWrite(EN_left, LOW);
  digitalWrite(EN_right, LOW);
  Serial.println("Motors stopped.");
}

// sum of array
int sumArray(uint16_t arr[], int size)
{
  int sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += arr[i];
  }
  return sum;
}

// test pwm
void testPWM()
{

  Serial.println("Testing PWM at BWD");
  for (int i = 120; i > 10; i -= 5)
  {
    setLeftMotor(i);
    setRightMotor(i);

    Serial.print("PWM value: ");
    Serial.print(i);
    Serial.println();

    delay(1000);
  }

  Serial.println("Testing PWM at FWD");
  for (int i = 136; i < 250; i += 5)
  {
    setLeftMotor(i);
    setRightMotor(i);

    Serial.print("PWM value: ");
    Serial.print(i);
    Serial.println();

    delay(1000);
  }

  stopMotors();
  Serial.println("Testing complete.");
  delay(2000);
}
// Debug print
void printImportantInfo(uint16_t sensorValue[], int IRValues[])
{
  Serial.print("QTR Sensor Values: ");
  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValue[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("IR Sensor Values: ");
  for (int i = 0; i < IRCount; i++)
  {
    Serial.print(IRValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void loop()
{
  // Read QTR sensors
  uint16_t sensorValue[SensorCount];
  qtr.read(sensorValue, QTRReadMode::On);

  while (sumArray(sensorValue, SensorCount) < EDGE_THRESHOLD)
  {

    int IRValues[IRCount]; // Array to store IR sensor values

    for (int i = 0; i < IRCount; i++)
    { // Read IR sensors

      IRValues[i] = digitalRead(IRPins[i]);
    }

    // choose where to attack

    if (IRValues[0] == 1) // opponent is located at the back
    {
      // Attack backward
      setLeftMotor(ATTACK_SPEED_BWD);
      setRightMotor(ATTACK_SPEED_BWD);
    }
    else if (IRValues[1] == 1) // opponent is located at the left
    {
      // Attack left
      setLeftMotor(ATTACK_SPEED_BWD);
      setRightMotor(ATTACK_SPEED_FWD);
    }
    else if (IRValues[2] == 1 || IRValues[3] == 1 || IRValues[4] == 1) // opponent is located at the center
    {
      // Attack center
      setLeftMotor(ATTACK_SPEED_FWD);
      setRightMotor(ATTACK_SPEED_FWD);
    }
    else if (IRValues[5] == 1) // opponent is located at right
    {
      // Attack right
      setLeftMotor(ATTACK_SPEED_FWD);
      setRightMotor(ATTACK_SPEED_BWD);
    }
    else // no opponent detected
    {
      // Search for opponent
      setLeftMotor(SEARCH_SPEED_FWD);
      setRightMotor(SEARCH_SPEED_BWD);
    }

    qtr.read(sensorValue, QTRReadMode::On);
  }
}
