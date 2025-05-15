#include <stdio.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include "hardware/pwm.h"

//yes
// left motor driver red black
// right motor driver black red

/*===== PIN DEFINITIONS =====*/
// Motor control pins
#define EN_left 10
#define EN_right 8
#define PWM_left 11
#define PWM_right 9
#define startPin 12 // starter pin

// IR sensor pins for opponent detection (6 sensors)
// Arrangement: back, left, front-left, center, front-right, right
int IRPins[] = {13, 14, 6, 20, 7, 21};
int IRCount = 6;

// QTR sensor pins for edge detection (4 sensors)
const uint8_t QTRPins[] = {2, 3, 4, 5};
QTRSensors qtr;
int SensorCount = 4;

/*===== MOVEMENT SETTINGS =====*/
// Motor speeds (0-255, 128 is stop)
#define STOP_SPEED 128
#define ATTACK_SPEED_FWD 160  // Forward attack
#define ATTACK_SPEED_BWD 96   // Backward attack
#define RETREAT_SPEED_FWD 140 // Forward retreat
#define RETREAT_SPEED_BWD 116 // Backward retreat
#define SEARCH_SPEED_FWD 146  // Forward search
#define SEARCH_SPEED_BWD 110  // Backward search

// Search pattern timing (in milliseconds)
const unsigned int FORWARD_TIME = 250;  // Forward phase
const unsigned int BACKWARD_TIME = 250; // Backward phase
const unsigned int TURN_TIME = 5;     // Turn phase

/*===== THRESHOLDS =====*/
#define EDGE_THRESHOLD 400   // Value indicating ring edge
#define OPPONENT_THRESHOLD 1 // Value indicating opponent detected

/*===== GLOBAL VARIABLES =====*/
// PWM configuration
#define PWM_FREQ 3900
#define PWM_RESOLUTION 255
uint slice_left;
uint slice_right;

// Tracking variables
unsigned long lastSerialOutput = 0;
const unsigned long SERIAL_INTERVAL = 300; // Print interval
int lastIRValues[6] = {0};                 // Previous IR readings
unsigned long searchStartTime = 0;

/*===== SETUP FUNCTIONS =====*/
// Configure PWM on a pin
void setupPWM(uint pin, uint &slice)
{
  gpio_set_function(pin, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num(pin);

  float divider = (float)125000000 / (PWM_FREQ * (PWM_RESOLUTION + 1));
  divider = constrain(divider, 1.0f, 255.0f);
  pwm_set_clkdiv(slice, divider);
  pwm_set_wrap(slice, PWM_RESOLUTION);
  pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
  pwm_set_enabled(slice, true);
}

void setup()
{
  // Configure motor pins
  pinMode(EN_left, OUTPUT);
  pinMode(EN_right, OUTPUT);
  digitalWrite(EN_left, LOW);
  digitalWrite(EN_right, LOW);

  // Configure IR sensors
  for (int i = 0; i < IRCount; i++)
  {
    pinMode(IRPins[i], INPUT);
  }

  // Configure QTR sensors
  qtr.setTypeRC();
  qtr.setSensorPins(QTRPins, SensorCount);
  for (int i = 0; i < 300; i++)
  {
    qtr.calibrate();
  }

  // Configure start pin
  pinMode(startPin, INPUT);

  // Configure PWM for motors
  setupPWM(PWM_left, slice_left);
  setupPWM(PWM_right, slice_right);

  Serial.begin(115200);
  Serial.println("Sumo robot initialized. Waiting for start signal.");
}

/*===== MOTOR CONTROL =====*/
// Set left motor speed (0-255)
void setLeftMotor(int speed)
{
  digitalWrite(EN_left, HIGH);
  pwm_set_chan_level(slice_left, pwm_gpio_to_channel(PWM_left), speed);
}

// Set right motor speed (0-255)
void setRightMotor(int speed)
{
  digitalWrite(EN_right, HIGH);
  pwm_set_chan_level(slice_right, pwm_gpio_to_channel(PWM_right), speed);
}

// Stop both motors
void stopMotors()
{
  digitalWrite(EN_left, LOW);
  digitalWrite(EN_right, LOW);
  Serial.println("Motors stopped.");
}


void logMessage(const char *message);
void logStateChange(const char *message, int newState, int oldState);


// Search pattern state machine variables
unsigned long searchPatternTimer = 0;
int searchPatternState = 0; // 0=forward, 1=backward, 2=turn
int lastOpponentState = -1; // Track if we just came from detecting an opponent


/*===== MAIN LOOP =====*/
void loop()
{
  // // 1. Read sensors
  // // Uncomment to enable edge detection
  // /*
  // uint16_t sensorValue[SensorCount];
  // qtr.read(sensorValue, QTRReadMode::On);

  // // 2. Check for ring edge (highest priority)
  // if (sensorValue[0] < EDGE_THRESHOLD || sensorValue[3] < EDGE_THRESHOLD) {
  //   // Edge detected - back up and turn away
  //   setLeftMotor(RETREAT_SPEED_BWD);
  //   setRightMotor(RETREAT_SPEED_BWD);
  //   delay(200);
  //   setLeftMotor(RETREAT_SPEED_BWD);
  //   setRightMotor(RETREAT_SPEED_FWD);
  //   delay(150);
  //   return;
  // }
  

 //3. Read IR sensors for opponent detection
  int IRValues[IRCount] = {0};
  for (int i = 0; i < IRCount; i++)
  {
    IRValues[i] = digitalRead(IRPins[i]);
    Serial.print(IRValues[i]);
    Serial.print(" ");
  }

  Serial.println();


  // 4. Determine opponent position (0=none, 1=back, 2=left, 3=center, 4=right)
  int opponentState = 0;
  if (IRValues[0] == 1)
    opponentState = 1; // Back
  else if (IRValues[1] == 1)
    opponentState = 2; // Left
  else if (IRValues[2] == 1 || IRValues[3] == 1 || IRValues[4] == 1)
    opponentState = 3; // Center
  else if (IRValues[5] == 1)
    opponentState = 4; // Right

  // 5. React based on opponent position
  switch (opponentState)
{
  case 0: { // No opponent - search pattern
    unsigned long currentMillis = millis();
    
    // Reset search state if we just lost track of opponent
    if (lastOpponentState != 0) {
      searchPatternState = 0;
      searchPatternTimer = currentMillis;
    }
    
    // State machine for search pattern
    if (searchPatternState == 0) { // Forward phase
      setLeftMotor(SEARCH_SPEED_FWD);
      setRightMotor(SEARCH_SPEED_FWD - 1);
      
      if (currentMillis - searchPatternTimer >= FORWARD_TIME) {
        searchPatternState = 1;
        searchPatternTimer = currentMillis;
      }
    }
    else if (searchPatternState == 1) { // Backward phase
      setLeftMotor(SEARCH_SPEED_BWD);
      setRightMotor(SEARCH_SPEED_BWD - 1);
      
      if (currentMillis - searchPatternTimer >= BACKWARD_TIME) {
        searchPatternState = 2;
        searchPatternTimer = currentMillis;
      }
    }
    else if (searchPatternState == 2) { // Turn phase
      setLeftMotor(SEARCH_SPEED_FWD);
      setRightMotor(SEARCH_SPEED_BWD);
      
      if (currentMillis - searchPatternTimer >= TURN_TIME) {
        searchPatternState = 0;
        searchPatternTimer = currentMillis;
      }
    }
    
    break;
  }
   case 1: { // Opponent at back
    // Faster backward attack with slight direction bias based on last attack
    if (lastOpponentState == 2) { // If opponent was recently at left
      setLeftMotor(SEARCH_SPEED_BWD - 5); // Turn more sharply
      setRightMotor(SEARCH_SPEED_BWD + 5);
    } else if (lastOpponentState == 4) { // If opponent was recently at right
      setLeftMotor(SEARCH_SPEED_BWD + 5);
      setRightMotor(SEARCH_SPEED_BWD - 5); // Turn more sharply
    } else {
      // Faster backward attack
      setLeftMotor(SEARCH_SPEED_BWD - 10); // More speed (lower value = faster backward)
      setRightMotor(SEARCH_SPEED_BWD - 10);
    }
    logStateChange("Attacking backward", IRValues[0], lastIRValues[0]);
    break;
  }
  case 2: { // Opponent at left
    // More aggressive left turn with initial burst

    setLeftMotor(SEARCH_SPEED_BWD); // Higher backward speed
    setRightMotor(SEARCH_SPEED_FWD + 2); // Higher forward speed
    logStateChange("Attacking left", IRValues[1], lastIRValues[1]);
    break;
  }
  case 3: { // Opponent at center
    // Optimize frontal attack with directional bias based on which sensors are active
    if (IRValues[2] && !IRValues[4]) {
      // Opponent slightly left of center - adjust trajectory
      setLeftMotor(SEARCH_SPEED_FWD + 15);
      setRightMotor(SEARCH_SPEED_FWD + 20); // Right motor faster to turn slightly left
    } else if (!IRValues[2] && IRValues[4]) {
      // Opponent slightly right of center - adjust trajectory
      setLeftMotor(SEARCH_SPEED_FWD + 25); // Left motor faster to turn slightly right
      setRightMotor(SEARCH_SPEED_FWD + 20);
    } else {
      // Direct center attack with max speed
      setLeftMotor(SEARCH_SPEED_FWD + 30); // Much higher speed
      setRightMotor(SEARCH_SPEED_FWD + 30);
    }
    logStateChange("Attacking center",
                  (IRValues[2] || IRValues[3] || IRValues[4]),
                  (lastIRValues[2] || lastIRValues[3] || lastIRValues[4]));
    break;
  }
  case 4: { // Opponent at right
    // More aggressive right turn with initial burst
    setLeftMotor(SEARCH_SPEED_FWD + 4); // Higher forward speed
    setRightMotor(SEARCH_SPEED_BWD); // Higher backward speed
    logStateChange("Attacking right", IRValues[5], lastIRValues[5]);
    break;
  }
}

// Update last opponent state for next loop
lastOpponentState = opponentState;


// setLeftMotor(SEARCH_SPEED_FWD);
// setRightMotor(SEARCH_SPEED_FWD - 1);

// setLeftMotor(SEARCH_SPEED_BWD);
// setRightMotor(SEARCH_SPEED_BWD - 1);


}

/*===== HELPER FUNCTIONS =====*/
// Log a message (limited by SERIAL_INTERVAL)
void logMessage(const char *message)
{
  if (millis() - lastSerialOutput > SERIAL_INTERVAL)
  {
    Serial.println(message);
    lastSerialOutput = millis();
  }
}

// Log a message only when state changes
void logStateChange(const char *message, int newState, int oldState)
{
  if (newState != oldState && millis() - lastSerialOutput > SERIAL_INTERVAL)
  {
    Serial.println(message);
    lastSerialOutput = millis();
  }
}

// Print sensor values for debugging
void printImportantInfo(uint16_t sensorValue[], int IRValues[])
{
  Serial.print("QTR: ");
  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValue[i]);
    Serial.print(" ");
  }

  Serial.print("| IR: ");
  for (int i = 0; i < IRCount; i++)
  {
    Serial.print(IRValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}