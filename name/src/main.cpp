#include <stdio.h>
#include <QTRSensors.h>
#include <Arduino.h>

/*DISCLAIMER*/
// For Motor Driver L we use these colors: GND Black, EN Brown, PWM Red
// For Motor Driver R we use these colors: GND White, EN Purple, PWM Blue

/*Pin Definition*/
#define EN_left 3  // Enable Pin for Motor Driver L Connected to Digital 3
#define EN_right 2 // Enable Pin for Motor Driver R Connected to Digital 2

#define PWM_left 26  // PWM Pin for Motor Driver L Connected to Digital 26
#define PWM_right 25 // PWM Pin for Motor Driver R Connected to Digital 25

// Define IR pins for opponent detection
int IRPins[] = {21, 22, 23, 24,25}; // Update with your actual pin numbers for all 4 sensors
int IRCount = 5;                 // Number of IR sensors

// Define QTR sensor pins for edge detection
const uint8_t QTRPins[] = {15, 16, 17, 18};
QTRSensors qtr;
int SensorCount = 4;

#define startPin 19 // starter pin

// Define threshold values
#define EDGE_THRESHOLD 500   // Value that indicates ring edge - adjust based on your QTR sensor readings
#define OPPONENT_THRESHOLD 1 // Value that indicates opponent detected - adjust based on your IR sensor

// Define motor speed constants
#define ATTACK_SPEED_FWD 252 // Full forward (your motor setup uses 252 for full forward)
#define RETREAT_SPEED_BWD 2  // Full backward (your motor setup uses 2 for full backward)
#define STOP_SPEED 128       // Stop (your motor setup uses 128 for stop)
#define SEARCH_SPEED_FWD 200 // Search forward speed
#define SEARCH_SPEED_BWD 50  // Search backward speed

// States for state machine
enum RobotState
{
  WAIT_FOR_START,
  SEARCH,
  ATTACK,
  RETREAT,
  TURN_LEFT,
  TURN_RIGHT,
  STOP
};

RobotState currentState = WAIT_FOR_START; // default state
unsigned long stateTimer = 0;

void setup()
{
  // Setting up motor driver pins
  pinMode(EN_left, OUTPUT);
  pinMode(EN_right, OUTPUT);

  digitalWrite(EN_left, LOW);  // Enable Pin set low for avoiding false startups
  digitalWrite(EN_right, LOW); // Enable Pin set low for avoiding false startups

  // Setting up IR pins for opponent detection
  for (int i = 0; i < IRCount; i++)
  {
    pinMode(IRPins[i], INPUT);
  }

  // Setting up QTR pins for edge detection
  qtr.setTypeRC();
  qtr.setSensorPins(QTRPins, SensorCount);

  // Calibrate QTR sensors
  // for (int i = 0; i < 300; i++) {
  //   qtr.calibrate();
  // }

  // Setting up start pin
  pinMode(startPin, INPUT);

  Serial.begin(9600);
  Serial.println("Sumo robot initialized. Waiting for start signal.");
}

// Function to set left motor speed
void setLeftMotor(int speed)
{
  digitalWrite(EN_left, HIGH);  // Enable the motor
  analogWrite(PWM_left, speed); // Set the speed
}

// Function to set right motor speed
void setRightMotor(int speed)
{
  digitalWrite(EN_right, HIGH);  // Enable the motor
  analogWrite(PWM_right, speed); // Set the speed
}

// Function to stop both motors
void stopMotors()
{
  digitalWrite(EN_left, LOW);
  digitalWrite(EN_right, LOW);
  analogWrite(PWM_left, STOP_SPEED);
  analogWrite(PWM_right, STOP_SPEED);
}

void loop()
{
  // Read sensor values
  uint16_t sensorValues[SensorCount];
  qtr.readLineBlack(sensorValues);

  // Read all IR sensors for opponent detection
  int opponentDetected[IRCount];
  bool anyOpponentDetected = false;

  for (int i = 0; i < IRCount; i++)
  {
    opponentDetected[i] = digitalRead(IRPins[i]); // 1 when opponent detected
    if (opponentDetected[i])
      anyOpponentDetected = true;
  }

  // Calculate which direction the opponent is in (left vs right)
  bool opponentLeft = (opponentDetected[0] || opponentDetected[1]);  // Left side sensors
  bool opponentRight = (opponentDetected[4] || opponentDetected[3]); // Right side sensors
  bool opponentFront = (opponentDetected[2]); // Front sensor

  // Check start button/sensor
  int starter = digitalRead(startPin);

  // State machine
  switch (currentState)
  {

  case WAIT_FOR_START:

    if (starter)
    {
      currentState = SEARCH;
      Serial.println("Robot started! Beginning search pattern.");
      // Wait 5 seconds before starting (tournament rules often require this)
      delay(5000);
    }
    break;

  case SEARCH:
    // Search pattern: rotate while looking for opponent
    int sensorValue = sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3];
    // Check for edge detection
    if (sensorValue < EDGE_THRESHOLD)
    {
      // Edge detected, retreat
      currentState = STOP;
      stateTimer = millis() + 500; // Retreat for 500ms
      Serial.println("Edge detected during search! Retreating.");
    }
    else if (anyOpponentDetected)
    {
      // Opponent found, attack!
      currentState = ATTACK;
      Serial.println("Opponent detected! Attacking!");

      // Decide which way to turn based on where the opponent is detected
      if (opponentLeft && !opponentRight)
      {
        // Turn left toward opponent
        setLeftMotor(SEARCH_SPEED_BWD);  // Backward
        setRightMotor(ATTACK_SPEED_FWD); // Forward
        Serial.println("Turning left toward opponent");
      }
      else if (opponentRight && !opponentLeft)
      {
        // Turn right toward opponent
        setLeftMotor(ATTACK_SPEED_FWD);  // Forward
        setRightMotor(SEARCH_SPEED_BWD); // Backward
        Serial.println("Turning right toward opponent");
      }
      else if (opponentFront)
      {
        // Opponent is directly ahead
        setLeftMotor(ATTACK_SPEED_FWD);
        setRightMotor(ATTACK_SPEED_FWD);
        Serial.println("Opponent directly ahead");
      }
      else
      {
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // No opponent detected but anyOpponentDetected was true?
        // This is a fallback that shouldn't normally be reached
        setLeftMotor(ATTACK_SPEED_FWD);
        setRightMotor(ATTACK_SPEED_FWD);
      }
    }
    else
    {
      // Continue searching - rotate in place
      setLeftMotor(SEARCH_SPEED_FWD);  // Forward
      setRightMotor(SEARCH_SPEED_BWD); // Backward
    }
    break;

  case ATTACK:
    // Full speed ahead to push opponent
    if (sensorValues[0] < EDGE_THRESHOLD || sensorValues[1] < EDGE_THRESHOLD ||
        sensorValues[2] < EDGE_THRESHOLD || sensorValues[3] < EDGE_THRESHOLD)
    {
      // Edge detected, retreat
      currentState = STOP;
      stateTimer = millis() + 500; // Retreat for 500ms
      Serial.println("Edge detected during attack! Retreating.");
    }
    else if (!anyOpponentDetected)
    {
      // Lost sight of opponent, go back to search
      currentState = SEARCH;
      Serial.println("Opponent lost! Searching again.");
    }
    else
    {
      // Continue attacking, adjusting direction based on sensor readings
      if (opponentLeft && !opponentRight)
      {
        // Turn left toward opponent
        setLeftMotor(SEARCH_SPEED_FWD);  // Slightly slower
        setRightMotor(ATTACK_SPEED_FWD); // Full speed
      }
      else if (opponentRight && !opponentLeft)
      {
        // Turn right toward opponent
        setLeftMotor(ATTACK_SPEED_FWD);  // Full speed
        setRightMotor(SEARCH_SPEED_FWD); // Slightly slower
      }
      else
      {
        // Opponent is straight ahead
        setLeftMotor(ATTACK_SPEED_FWD);
        setRightMotor(ATTACK_SPEED_FWD);
      }
    }
    break;

  case RETREAT:
    // Back away from edge
    setLeftMotor(RETREAT_SPEED_BWD);
    setRightMotor(RETREAT_SPEED_BWD);

    // Check if retreat time is over
    if (millis() > stateTimer)
    {
      // After retreating, turn away from the edge
      currentState = TURN_RIGHT;   // You can randomize this or use additional sensors
      stateTimer = millis() + 750; // Turn for 750ms
      Serial.println("Retreat complete. Turning away from edge.");
    }
    break;

  case TURN_LEFT:
    // Turn left in place
    setLeftMotor(RETREAT_SPEED_BWD);
    setRightMotor(ATTACK_SPEED_FWD);

    // If turn is complete, go back to search
    if (millis() > stateTimer)
    {
      currentState = SEARCH;
      Serial.println("Turn complete. Resuming search.");
    }
    break;

  case TURN_RIGHT:
    // Turn right in place
    setLeftMotor(ATTACK_SPEED_FWD);
    setRightMotor(RETREAT_SPEED_BWD);

    // If turn is complete, go back to search
    if (millis() > stateTimer)
    {
      currentState = SEARCH;
      Serial.println("Turn complete. Resuming search.");
    }
    break;

  case STOP:
    // Stop motors
    stopMotors();
    Serial.println("Robot stopped.");
    break;
  }

  // Debug output every 500ms to avoid flooding serial
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500)
  {
    Serial.print("State: ");
    Serial.print(currentState);
    Serial.print(" | Edge Sensor: ");
    Serial.print(sensorValues[0]);
    Serial.print(" | Opponent: ");
    Serial.println(anyOpponentDetected);

    lastDebugTime = millis();
  }
}