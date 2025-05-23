#include <stdio.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include <Wire.h>
#include "hardware/pwm.h"

//  left motor driver red black
//  right motor driver black red

/*===== PIN DEFINITIONS =====*/
// Motor control pins
#define EN_left 10
#define PWM_left 11

#define EN_right 8
#define PWM_right 9

#define startPin 12 // starter pin

#define TMP_1 26
#define TMP_2 27

// IR sensor pins for opponent detection (6 sensors)
// Arrangement: back, left, front-left, center, front-right, right
// pin 13 is disabled

int IRPins[] = {6, 14, 7, 20, 21};
int IRCount = 5;

// QTR sensor pins for edge detection (4 sensors)
const uint8_t QTRPins[] = {2, 3};
QTRSensors qtr;
int QTRSensorCount = 2;

/*===== MOVEMENT SETTINGS =====*/
#define STOP_SPEED 128
#define ATTACK_SPEED_FWD 200 // Forward attack
#define ATTACK_SPEED_BWD 56  // Backward attack

#define RETREAT_SPEED_FWD 200 // Forward retreat
#define RETREAT_SPEED_BWD 56  // Backward retreat

#define SEARCH_SPEED_FWD 146 // Forward search +5 for right curve
#define SEARCH_SPEED_BWD 110 // Backward search - 8 for right curve

// Optimized turning parameters
#define TURN_SPEED_AGGRESSIVE 15 // For sharp turns
#define TURN_SPEED_MODERATE 10   // For moderate turns
#define TURN_SPEED_GENTLE 5      // For gentle turning

#define BACK_TURN_TIME 300 // Reduced from 350ms for more precise 180° turns

// Search pattern timing (in milliseconds)
const unsigned int FORWARD_TIME = 250;  // Forward phase
const unsigned int BACKWARD_TIME = 300; // Backward phase
const unsigned int TURN_TIME = 100;     // Turn phase

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
int lastIRValues[5] = {0};                 // Previous IR readings (not 6)
unsigned long searchStartTime = 0;

// Back attack state machine variables
unsigned long backAttackTimer = 0;
int backAttackState = 0; // 0=turn, 1=attack

// Global variables for improved sensor debouncing
#define IR_DEBOUNCE_COUNT 2                // Number of consecutive readings to confirm detection
int IRConfirmedCount[5] = {0, 0, 0, 0, 0}; // Counters for debouncing (not 6)

// Search pattern state machine variables
unsigned long searchPatternTimer = 0;
int searchPatternState = 0; // 0=forward, 1=backward, 2=turn
int lastOpponentState = -1; // Track if we just came from detecting an opponent

// Add a variable to track time spent in states to detect if stuck
unsigned long stateEntryTime = 0;
#define MAX_STATE_TIME 2000 // Maximum time to spend in any state (ms)

// Temperature safety check variables
unsigned long lastTempCheckTime = 0;
const unsigned long TEMP_CHECK_INTERVAL = 1000; // Check temperature every 1 second

// Add these to your GLOBAL VARIABLES section
unsigned long turnStabilizeTimer = 0;
bool stabilizingTurn = false;

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
  qtr.setSensorPins(QTRPins, QTRSensorCount);

  for (int i = 0; i < 300; i++)
  {
    qtr.calibrate();
  }

  // Configure start pin
  pinMode(startPin, INPUT);

  // Configure PWM for motors
  setupPWM(PWM_left, slice_left);
  setupPWM(PWM_right, slice_right);

  // Configure temp sensors
  pinMode(TMP_1, INPUT);
  pinMode(TMP_2, INPUT);

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

// Corrected temperature mapping with proper offset and inversion
float mapTemperature(int analogValue)
{
  // Prevent division by zero or invalid readings
  if (analogValue <= 0 || analogValue >= 1023)
    return -10.0;

  // Calculate thermistor resistance using voltage divider formula
  float resistance = 10000.0 * ((1023.0 / (float)analogValue) - 1.0);

  // Use the B-parameter equation (simplified Steinhart-Hart)
  float steinhart = log(resistance / 10000.0);
  steinhart /= 3950.0;
  steinhart += 1.0 / 298.15;
  steinhart = 1.0 / steinhart;
  float celsius = steinhart - 273.15;

  // Apply 20°C correction offset

  return map(celsius, 50, -50, 24.0, 120.0) - 10; // Map to 0-1 range
}

void logMessage(const char *message);
void logStateChange(const char *message, int newState, int oldState);
void printImportantInfo(uint16_t sensorValue[], int IRValues[]);
void mainFUNC();

/*===== MAIN LOOP =====*/
void loop()
{

  mainFUNC();
  // delay(500); // Small delay to prevent overwhelming the loop
}

/*===== HELPER FUNCTIONS =====*/
// Log a message
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
  for (int i = 0; i < QTRSensorCount; i++)
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

  Serial.print("Motor temps: ");
  Serial.print(analogRead(TMP_1));
  Serial.print(" ");

  Serial.println();
}

void mainFUNC()
{
  unsigned long currentMillis = millis();

  // Temperature safety check - runs every 1 second
  if (currentMillis - lastTempCheckTime >= TEMP_CHECK_INTERVAL)
  {
    lastTempCheckTime = currentMillis;

    // Read both temperature sensors
    int tempValue1 = analogRead(TMP_1);
    int tempValue2 = analogRead(TMP_2);
    float temp1 = mapTemperature(tempValue1);
    float temp2 = mapTemperature(tempValue2);

    // Log current temperatures
    // Serial.print("TEMP CHECK: ");
    // Serial.print(temp1);
    // Serial.print("°C, ");
    // Serial.print(temp2);
    // Serial.println("°C");

    // Safety cutoff - if either temperature exceeds threshold
    if (temp1 > 70.0 || temp2 > 70.0)
    {
      // Emergency stop
      stopMotors();
      Serial.println("EMERGENCY STOP");
      // Break out of the main function
      return;
    }
  }

  // Enable edge detection - using non-blocking approach
  uint16_t sensorValue[QTRSensorCount];
  qtr.read(sensorValue, QTRReadMode::On);

  // Check for ring edge (highest priority)
  if (sensorValue[0] < EDGE_THRESHOLD || sensorValue[1] < EDGE_THRESHOLD)
  {
    // Edge detected - back up and turn away
    static unsigned long edgeTimer = 0;
    static int edgeState = 0;
    static bool leftEdge = false; // Remember which edge was detected
    static bool rightEdge = false;

    // Remember which sensors detected the edge
    if (edgeState == 0)
    {
      leftEdge = (sensorValue[1] < EDGE_THRESHOLD);
      rightEdge = (sensorValue[0] < EDGE_THRESHOLD);

      // Start backup sequence
      setLeftMotor(RETREAT_SPEED_BWD);
      setRightMotor(RETREAT_SPEED_BWD - 8);

      edgeTimer = currentMillis;
      edgeState = 1;
      logMessage("Edge detected - backing up");
    }
    // Backup phase (State 1)
    else if (edgeState == 1 && currentMillis - edgeTimer >= 200)
    {
      // After backing up for 200ms, start turning
      edgeTimer = currentMillis;
      edgeState = 2;

      // Log which direction we're turning
      if (leftEdge && !rightEdge)
      {
        logMessage("Left edge - turning right");
      }
      else if (rightEdge && !leftEdge)
      {
        logMessage("Right edge - turning left");
      }
      else
      {
        logMessage("Both edges - turning right");
      }
    }
    // Turn phase (State 2)
    else if (edgeState == 2 && currentMillis - edgeTimer >= 70)
    {
      // Turn completed, reset state
      edgeState = 0;
      logMessage("Edge recovery complete");
    }
    // Execute the turn based on which edge was detected
    else if (edgeState == 2)
    {
      if (leftEdge && !rightEdge)
      {
        // Left sensor detected edge, turn left
        setLeftMotor(RETREAT_SPEED_BWD - TURN_SPEED_AGGRESSIVE);
        setRightMotor(RETREAT_SPEED_FWD + TURN_SPEED_AGGRESSIVE);
      }
      else
      {
        // Right sensor detected edge or both sensors, turn right
        setLeftMotor(RETREAT_SPEED_FWD + TURN_SPEED_AGGRESSIVE);
        setRightMotor(RETREAT_SPEED_BWD - TURN_SPEED_AGGRESSIVE);
      }
    }

    // Skip rest of logic while handling edge
    return;
  }

  // Read IR sensors with debouncing for more reliable detection
  int IRValues[IRCount] = {0};
  for (int i = 0; i < IRCount; i++)
  {
    int rawValue = digitalRead(IRPins[i]);

    // Debounce readings
    if (rawValue == 1)
    {
      IRConfirmedCount[i]++;
      if (IRConfirmedCount[i] >= IR_DEBOUNCE_COUNT)
      {
        IRValues[i] = 1;
      }
    }
    else
    {
      IRConfirmedCount[i] = 0;
      IRValues[i] = 0;
    }

    Serial.print(IRValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Determine opponent position (0=none, 1=back, 2=left, 3=center, 4=right)
  int opponentState = 0;
  if (IRValues[0] == 1)
    opponentState = 1; // Back
  else if (IRValues[1] == 1)
    opponentState = 2; // Left
  else if (IRValues[2] == 1 || IRValues[3] == 1)
    opponentState = 3; // Center
  else if (IRValues[4] == 1)
    opponentState = 4; // Right

  // Reset state entry timer if state changed
  if (opponentState != lastOpponentState)
  {
    stateEntryTime = currentMillis;
  }

  // Check if stuck in a state too long - reset if needed
  if (currentMillis - stateEntryTime > MAX_STATE_TIME)
  {
    // Force a state change if stuck too long
    backAttackState = 0;
    searchPatternState = 0;
    stateEntryTime = currentMillis;
    // logMessage("WARNING: Reset due to state timeout");
  }

  // React based on opponent position
  switch (opponentState)
  {
  case 0:
  { // No opponent - search pattern
    // Reset search state if we just lost track of opponent
    if (lastOpponentState != 0)
    {
      searchPatternState = 0;
      searchPatternTimer = currentMillis;
    }

    // State machine for search pattern
    if (searchPatternState == 0)
    {                                      // Forward phase
      setLeftMotor(SEARCH_SPEED_FWD);      // Slightly faster
      setRightMotor(SEARCH_SPEED_FWD + 5); // Slight curve to search more area

      if (currentMillis - searchPatternTimer >= FORWARD_TIME)
      {
        searchPatternState = 1;
        searchPatternTimer = currentMillis;
      }
    }
    else if (searchPatternState == 1)
    {                                      // Backward phase
      setLeftMotor(SEARCH_SPEED_BWD);      // Slightly faster
      setRightMotor(SEARCH_SPEED_BWD - 8); // Slight curve for better coverage

      if (currentMillis - searchPatternTimer >= BACKWARD_TIME)
      {
        searchPatternState = 2;
        searchPatternTimer = currentMillis;
      }
    }
    else if (searchPatternState == 2)
    {                                      // Turn phase - wider turn
      setLeftMotor(SEARCH_SPEED_FWD + 20); // Faster spin
      setRightMotor(SEARCH_SPEED_BWD);     // Faster spin

      if (currentMillis - searchPatternTimer >= TURN_TIME)
      { // Slightly longer turn
        searchPatternState = 0;
        searchPatternTimer = currentMillis;
      }
    }
    break;
  }

  case 1:
  { // Opponent at back - optimized 180° turn
    // If we just detected opponent at back
    if (lastOpponentState != 1)
    {
      backAttackState = 0;
      backAttackTimer = currentMillis;

      // Start with a quick reverse
      setLeftMotor(SEARCH_SPEED_BWD);
      setRightMotor(SEARCH_SPEED_BWD - 5); // Reduced turn bias for straighter backup

      logMessage("Back opponent - starting sequence");
    }
    else
    {
      // State machine with proper sequence
      if (backAttackState == 0 && currentMillis - backAttackTimer >= 250)
      {
        // After brief backup, start the 180° turn
        setLeftMotor(SEARCH_SPEED_FWD + TURN_SPEED_AGGRESSIVE);
        setRightMotor(SEARCH_SPEED_BWD - TURN_SPEED_AGGRESSIVE);
        backAttackState = 1; // Progress to state 1 (turning)
        backAttackTimer = currentMillis;
        logMessage("Back attack - turning 180°");
      }
      else if (backAttackState == 1 && currentMillis - backAttackTimer >= BACK_TURN_TIME)
      {
        // After turn completed, switch to attack state
        backAttackState = 2;
        backAttackTimer = currentMillis;
        logMessage("Back attack - turn complete, attacking");
      }
      else if (backAttackState == 2)
      {
        // Full speed ahead
        setLeftMotor(ATTACK_SPEED_FWD + 15);
        setRightMotor(ATTACK_SPEED_FWD + 20);
      }
    }
    break;
  }

  case 2:
  { // Opponent at left - more precise turning
    // More gradual turn with slightly reduced differential
    setLeftMotor(SEARCH_SPEED_BWD);
    setRightMotor(ATTACK_SPEED_FWD + TURN_SPEED_AGGRESSIVE + 40);

    // If we just detected the opponent on the left
    if (lastOpponentState != 2)
    {
      stabilizingTurn = true;
      turnStabilizeTimer = currentMillis;
    }

    // Add a stabilization phase to prevent overturning
    if (stabilizingTurn && currentMillis - turnStabilizeTimer >= 150)
    {
      stabilizingTurn = false;
    }

    logStateChange("Attacking left", IRValues[1], lastIRValues[1]);
    break;
  }

  case 3:
  { // Opponent at center - more precise directional control
    if (IRValues[2] && !IRValues[4])
    {
      // Opponent slightly left of center - gentler turn
      setLeftMotor(ATTACK_SPEED_FWD + 5);
      setRightMotor(ATTACK_SPEED_FWD + 20); // Reduced differential
    }
    else if (!IRValues[2] && IRValues[4])
    {
      // Opponent slightly right of center - gentler turn
      setLeftMotor(ATTACK_SPEED_FWD + 20); // Reduced differential
      setRightMotor(ATTACK_SPEED_FWD + 5);
    }
    else
    {
      // Direct center attack with balanced speed
      setLeftMotor(ATTACK_SPEED_FWD + 25);
      setRightMotor(ATTACK_SPEED_FWD + 30);
    }
    logStateChange("Attacking center",
                   (IRValues[2] || IRValues[3] || IRValues[4]),
                   (lastIRValues[2] || lastIRValues[3] || lastIRValues[4]));
    break;
  }

  case 4:
  { // Opponent at right - more precise turning
    // More gradual turn with slightly reduced differential
    setLeftMotor(ATTACK_SPEED_FWD + TURN_SPEED_AGGRESSIVE + 40);
    setRightMotor(SEARCH_SPEED_BWD);

    // If we just detected the opponent on the right
    if (lastOpponentState != 4)
    {
      stabilizingTurn = true;
      turnStabilizeTimer = currentMillis;
    }

    // Add a stabilization phase to prevent overturning
    if (stabilizingTurn && currentMillis - turnStabilizeTimer >= 150)
    {
      stabilizingTurn = false;
    }

    logStateChange("Attacking right", IRValues[4], lastIRValues[4]);
    break;
  }
  }

  // Update last opponent state for next loop
  lastOpponentState = opponentState;

  // Update IR values for next comparison
  for (int i = 0; i < IRCount; i++)
  {
    lastIRValues[i] = IRValues[i];
  }
}
