#include <stdio.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include "hardware/pwm.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

// Import our constants
#include "constants.h"

// Global objects
QTRSensors qtr;

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t decisionTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;

// Semaphores for protecting shared data
SemaphoreHandle_t sensorMutex = NULL;
SemaphoreHandle_t motorMutex = NULL;
SemaphoreHandle_t stateMutex = NULL;

// PWM configuration variables
uint slice_left;
uint slice_right;

// Shared sensor data (protected by sensorMutex)
struct SensorData {
  uint16_t qtrValues[SensorCount];
  int irValues[IRCount];
  bool edgeDetected;
  // Add temperature data
  float motorTemp1;
  float motorTemp2;
  // Add IR debounce counters
  int irConfirmedCount[IRCount];
};
SensorData sensorData;

// Shared motor commands (protected by motorMutex)
struct MotorCommands {
  int leftSpeed;
  int rightSpeed;
};
MotorCommands motorCommands;

// Shared state data (protected by stateMutex)
struct StateData {
  int opponentState;
  int lastOpponentState;
  int searchPatternState;
  unsigned long searchPatternTimer;
  bool robotStarted;
  // Add these new state variables
  int backAttackState;
  unsigned long backAttackTimer;
  unsigned long stateEntryTime;
};
StateData stateData;

// Function prototypes
void SensorTask(void *pvParameters);
void MotorControlTask(void *pvParameters);
void DecisionTask(void *pvParameters);
void ButtonTask(void *pvParameters);
void setupPWM(uint pin, uint &slice);
void setLeftMotor(int speed);
void setRightMotor(int speed);
void stopMotors();
void logMessage(const char *message);
void logStateChange(const char *message, int newState, int oldState);
float mapTemperature(int analogValue);
void printTemperatureInfo();

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) delay(10);
  Serial.println("Initializing Sumo Robot with FreeRTOS...");
  
  // Configure motor pins
  pinMode(EN_left, OUTPUT);
  pinMode(EN_right, OUTPUT);
  digitalWrite(EN_left, LOW);
  digitalWrite(EN_right, LOW);

  // Configure IR sensors
  for (int i = 0; i < IRCount; i++) {
    pinMode(IRPins[i], INPUT);
  }

  // Configure QTR sensors
  qtr.setTypeRC();
  qtr.setSensorPins(QTRPins, SensorCount);
  Serial.println("Calibrating QTR sensors...");
  for (int i = 0; i < 300; i++) {
    qtr.calibrate();
    delay(10);
  }
  Serial.println("Calibration complete");

  // Configure start pin
  pinMode(startPin, INPUT);

  // Configure PWM for motors
  setupPWM(PWM_left, slice_left);
  setupPWM(PWM_right, slice_right);
  
  // Configure temperature sensors
  pinMode(TMP_1, INPUT);
  pinMode(TMP_2, INPUT);
  
  // Initialize shared data structures
  sensorData.edgeDetected = false;
  for (int i = 0; i < SensorCount; i++) {
    sensorData.qtrValues[i] = 0;
  }
  for (int i = 0; i < IRCount; i++) {
    sensorData.irValues[i] = 0;
  }
  
  motorCommands.leftSpeed = STOP_SPEED;
  motorCommands.rightSpeed = STOP_SPEED;
  
  stateData.opponentState = 0;
  stateData.lastOpponentState = -1;
  stateData.searchPatternState = 0;
  stateData.searchPatternTimer = 0;
  stateData.robotStarted = false;
  stateData.backAttackState = 0;
  stateData.backAttackTimer = 0;
  stateData.stateEntryTime = 0;

  // Initialize IR debounce counters
  for (int i = 0; i < IRCount; i++) {
    sensorData.irConfirmedCount[i] = 0;
  }

  // Create mutexes
  sensorMutex = xSemaphoreCreateMutex();
  motorMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();
  
  // Ensure mutexes were created successfully
  if (sensorMutex == NULL || motorMutex == NULL || stateMutex == NULL) {
    Serial.println("ERROR: Failed to create one or more mutexes");
    while (1); // Hang if mutex creation failed
  }

  // Create tasks
  BaseType_t xReturned;
  
  xReturned = xTaskCreate(
    SensorTask,
    "SensorTask",
    STACK_SIZE_SENSOR,
    NULL,
    PRIORITY_SENSOR_TASK,
    &sensorTaskHandle
  );
  if (xReturned != pdPASS) {
    Serial.println("ERROR: Failed to create sensor task");
  }
  
  xReturned = xTaskCreate(
    MotorControlTask,
    "MotorTask",
    STACK_SIZE_MOTOR,
    NULL,
    PRIORITY_MOTOR_TASK,
    &motorTaskHandle
  );
  if (xReturned != pdPASS) {
    Serial.println("ERROR: Failed to create motor control task");
  }
  
  xReturned = xTaskCreate(
    DecisionTask,
    "DecisionTask",
    STACK_SIZE_DECISION,
    NULL,
    PRIORITY_DECISION_TASK,
    &decisionTaskHandle
  );
  if (xReturned != pdPASS) {
    Serial.println("ERROR: Failed to create decision task");
  }
  
  xReturned = xTaskCreate(
    ButtonTask,
    "ButtonTask",
    STACK_SIZE_BUTTON,
    NULL,
    PRIORITY_BUTTON_TASK,
    &buttonTaskHandle
  );
  if (xReturned != pdPASS) {
    Serial.println("ERROR: Failed to create button task");
  }
  
  Serial.println("Starting FreeRTOS scheduler...");
  
  // Start the scheduler
  vTaskStartScheduler();
  
  // Should never reach here if scheduler started correctly
  Serial.println("ERROR: Scheduler failed to start!");
  while (1);
}

void loop() {
  // Empty - not used with FreeRTOS
}

// Task to read sensors and update shared sensor data
void SensorTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long lastTempCheckTime = 0;
  
  for (;;) {
    // Get current time
    unsigned long currentMillis = millis();
    
    // Read QTR sensors
    uint16_t qtrValues[SensorCount];
    qtr.read(qtrValues);
    
    // Read IR sensors with debouncing
    int rawIrValues[IRCount];
    int debouncedIrValues[IRCount];
    
    for (int i = 0; i < IRCount; i++) {
      rawIrValues[i] = digitalRead(IRPins[i]);
      
      // Apply debouncing logic
      if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
        if (rawIrValues[i] == 1) {
          sensorData.irConfirmedCount[i]++;
          if (sensorData.irConfirmedCount[i] >= IR_DEBOUNCE_COUNT) {
            debouncedIrValues[i] = 1;
          } else {
            debouncedIrValues[i] = 0;
          }
        } else {
          sensorData.irConfirmedCount[i] = 0;
          debouncedIrValues[i] = 0;
        }
        xSemaphoreGive(sensorMutex);
      }
    }
    
    // Temperature safety check - runs every TEMP_CHECK_INTERVAL
    if (currentMillis - lastTempCheckTime >= TEMP_CHECK_INTERVAL) {
      lastTempCheckTime = currentMillis;
      
      // Read temperature sensors
      int tempValue1 = analogRead(TMP_1);
      int tempValue2 = analogRead(TMP_2);
      float temp1 = mapTemperature(tempValue1);
      float temp2 = mapTemperature(tempValue2);
      
      // Update shared sensor data with temperature values
      if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
        sensorData.motorTemp1 = temp1;
        sensorData.motorTemp2 = temp2;
        xSemaphoreGive(sensorMutex);
      }
      
      // Safety cutoff - if either temperature exceeds threshold
      if (temp1 > MAX_TEMP_THRESHOLD || temp2 > MAX_TEMP_THRESHOLD) {
        // Emergency stop via motor commands
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          motorCommands.leftSpeed = STOP_SPEED;
          motorCommands.rightSpeed = STOP_SPEED;
          xSemaphoreGive(motorMutex);
        }
        
        // Log the emergency stop
        Serial.println("!!! EMERGENCY STOP - MOTOR OVERHEATING !!!");
      }
    }
    
    // Determine if edge detected
    bool edgeDetected = false;
    for (int i = 0; i < SensorCount; i++) {
      if (qtrValues[i] < EDGE_THRESHOLD) {
        edgeDetected = true;
        break;
      }
    }
    
    // Update shared sensor data with mutex protection
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < SensorCount; i++) {
        sensorData.qtrValues[i] = qtrValues[i];
      }
      for (int i = 0; i < IRCount; i++) {
        sensorData.irValues[i] = debouncedIrValues[i];
      }
      sensorData.edgeDetected = edgeDetected;
      
      xSemaphoreGive(sensorMutex);
    }
    
    // Delay until next sensor reading
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_READ_DELAY));
  }
}

// Task to control motors based on motor commands
void MotorControlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int leftSpeed = STOP_SPEED;
  int rightSpeed = STOP_SPEED;
  
  for (;;) {
    // Get latest motor commands with mutex protection
    if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
      leftSpeed = motorCommands.leftSpeed;
      rightSpeed = motorCommands.rightSpeed;
      
      xSemaphoreGive(motorMutex);
    }
    
    // Apply motor commands
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
    
    // Delay until next motor update
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_UPDATE_DELAY));
  }
}

// Task for decision making logic
void DecisionTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool robotActive = false;
  bool edgeDetected = false;
  int irValues[IRCount];
  int opponentState = 0;
  int lastOpponentState = -1;
  int searchPatternState = 0;
  unsigned long searchPatternTimer = 0;
  int backAttackState = 0;
  unsigned long backAttackTimer = 0;
  unsigned long stateEntryTime = 0;
  
  for (;;) {
    unsigned long currentMillis = millis();
    
    // First check if robot should be active
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      robotActive = stateData.robotStarted;
      xSemaphoreGive(stateMutex);
    }
    
    if (!robotActive) {
      // Robot not started yet, skip decision logic
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DECISION_UPDATE_DELAY));
      continue;
    }
    
    // Get latest sensor data
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      edgeDetected = sensorData.edgeDetected;
      for (int i = 0; i < IRCount; i++) {
        irValues[i] = sensorData.irValues[i];
      }
      xSemaphoreGive(sensorMutex);
    }
    
    // Get state information
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      lastOpponentState = stateData.lastOpponentState;
      searchPatternState = stateData.searchPatternState;
      searchPatternTimer = stateData.searchPatternTimer;
      backAttackState = stateData.backAttackState;
      backAttackTimer = stateData.backAttackTimer;
      stateEntryTime = stateData.stateEntryTime;
      xSemaphoreGive(stateMutex);
    }
    
    // Check if stuck in a state too long - reset if needed
    if (currentMillis - stateEntryTime > MAX_STATE_TIME) {
      // Force a state change if stuck too long
      backAttackState = 0;
      searchPatternState = 0;
      stateEntryTime = currentMillis;
    }
    
    // 1. Edge detection (highest priority)
    if (edgeDetected) {
      if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
        motorCommands.leftSpeed = RETREAT_SPEED_BWD - 5; // Faster retreat
        motorCommands.rightSpeed = RETREAT_SPEED_BWD - 5;
        xSemaphoreGive(motorMutex);
      }
      
      // Wait a bit then turn away from edge
      vTaskDelay(pdMS_TO_TICKS(200));
      
      if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
        motorCommands.leftSpeed = RETREAT_SPEED_BWD - 10; // More aggressive turn
        motorCommands.rightSpeed = RETREAT_SPEED_FWD + 10;
        xSemaphoreGive(motorMutex);
      }
      
      vTaskDelay(pdMS_TO_TICKS(150));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DECISION_UPDATE_DELAY));
      continue;
    }
    
    // 2. Determine opponent position
    opponentState = 0;
    if (irValues[0] == 1) {
      opponentState = 1; // Back
    }
    else if (irValues[1] == 1) {
      opponentState = 2; // Left
    }
    else if (irValues[2] == 1 || irValues[3] == 1 || irValues[4] == 1) {
      opponentState = 3; // Center
    }
    else if (irValues[5] == 1) {
      opponentState = 4; // Right
    }
    
    // Reset state entry timer if state changed
    if (opponentState != lastOpponentState) {
      stateEntryTime = currentMillis;
    }
    
    // 3. React based on opponent position
    switch (opponentState) {
      case 0: { // No opponent - search pattern
        // Reset search state if we just lost track of opponent
        if (lastOpponentState != 0) {
          searchPatternState = 0;
          searchPatternTimer = currentMillis;
        }
        
        // State machine for search pattern
        if (searchPatternState == 0) { // Forward phase
          if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            motorCommands.leftSpeed = SEARCH_SPEED_FWD + 5;  // Slightly faster
            motorCommands.rightSpeed = SEARCH_SPEED_FWD + 3; // Slight curve to search more area
            xSemaphoreGive(motorMutex);
          }
          
          if (currentMillis - searchPatternTimer >= FORWARD_TIME) {
            searchPatternState = 1;
            searchPatternTimer = currentMillis;
          }
        }
        else if (searchPatternState == 1) { // Backward phase
          if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            motorCommands.leftSpeed = SEARCH_SPEED_BWD - 3; // Slightly faster
            motorCommands.rightSpeed = SEARCH_SPEED_BWD;    // Slight curve for better coverage
            xSemaphoreGive(motorMutex);
          }
          
          if (currentMillis - searchPatternTimer >= BACKWARD_TIME) {
            searchPatternState = 2;
            searchPatternTimer = currentMillis;
          }
        }
        else if (searchPatternState == 2) { // Turn phase - wider turn
          if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            motorCommands.leftSpeed = SEARCH_SPEED_FWD + 10;  // Faster spin
            motorCommands.rightSpeed = SEARCH_SPEED_BWD - 10; // Faster spin
            xSemaphoreGive(motorMutex);
          }
          
          if (currentMillis - searchPatternTimer >= TURN_TIME + 10) { // Slightly longer turn
            searchPatternState = 0;
            searchPatternTimer = currentMillis;
          }
        }
        break;
      }
      
      case 1: { // Opponent at back - improved state machine
        // If we just detected opponent at back, initialize turn
        if (lastOpponentState != 1) {
          backAttackState = 0;
          backAttackTimer = currentMillis;
          
          if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            // Start with brief reverse to gain space
            motorCommands.leftSpeed = SEARCH_SPEED_BWD - 5;
            motorCommands.rightSpeed = SEARCH_SPEED_BWD - 5;
            xSemaphoreGive(motorMutex);
          }
          
          logMessage("Detected opponent at back - preparing 180° turn");
        }
        else {
          // State machine for back opponent handling
          if (backAttackState == 0 && currentMillis - backAttackTimer >= BACK_ATTACK_REVERSE_TIME) {
            // After brief reverse, start fast 180° turn
            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
              motorCommands.leftSpeed = SEARCH_SPEED_FWD + 15;  // Faster turn
              motorCommands.rightSpeed = SEARCH_SPEED_BWD - 15; // Faster turn
              xSemaphoreGive(motorMutex);
            }
            backAttackState = 1;
            backAttackTimer = currentMillis;
            logMessage("Starting 180° turn");
          }
          else if (backAttackState == 0) {
            // Still in initial reverse
            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
              motorCommands.leftSpeed = SEARCH_SPEED_BWD - 5;
              motorCommands.rightSpeed = SEARCH_SPEED_BWD - 5;
              xSemaphoreGive(motorMutex);
            }
          }
          else if (backAttackState == 1 && currentMillis - backAttackTimer >= BACK_ATTACK_TURN_TIME) {
            // Turn complete - attack at full speed
            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
              motorCommands.leftSpeed = ATTACK_SPEED_FWD + 20;
              motorCommands.rightSpeed = ATTACK_SPEED_FWD + 20;
              xSemaphoreGive(motorMutex);
            }
            backAttackState = 2;
            logMessage("Turn complete - attacking");
          }
          else if (backAttackState == 1) {
            // Still turning
            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
              motorCommands.leftSpeed = SEARCH_SPEED_FWD + 15;
              motorCommands.rightSpeed = SEARCH_SPEED_BWD - 15;
              xSemaphoreGive(motorMutex);
            }
          }
          else if (backAttackState == 2) {
            // Continue attack
            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
              motorCommands.leftSpeed = ATTACK_SPEED_FWD + 20;
              motorCommands.rightSpeed = ATTACK_SPEED_FWD + 20;
              xSemaphoreGive(motorMutex);
            }
          }
        }
        break;
      }
      
      case 2: { // Opponent at left - more aggressive turn
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          motorCommands.leftSpeed = SEARCH_SPEED_BWD - 10;  // Faster backward
          motorCommands.rightSpeed = ATTACK_SPEED_FWD + 15; // Faster forward
          xSemaphoreGive(motorMutex);
        }
        
        logStateChange("Attacking left", irValues[1], (lastOpponentState == 2) ? 1 : 0);
        break;
      }
      
      case 3: { // Opponent at center - improved directional attack
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          if (irValues[2] && !irValues[4]) {
            // Opponent slightly left of center
            motorCommands.leftSpeed = ATTACK_SPEED_FWD + 10;
            motorCommands.rightSpeed = ATTACK_SPEED_FWD + 30; // More aggressive turn
          } else if (!irValues[2] && irValues[4]) {
            // Opponent slightly right of center
            motorCommands.leftSpeed = ATTACK_SPEED_FWD + 30; // More aggressive turn
            motorCommands.rightSpeed = ATTACK_SPEED_FWD + 10;
          } else {
            // Direct center attack with max speed
            motorCommands.leftSpeed = ATTACK_SPEED_FWD + 40; // Maximum attack speed
            motorCommands.rightSpeed = ATTACK_SPEED_FWD + 40;
          }
          xSemaphoreGive(motorMutex);
        }
        
        logStateChange("Attacking center",
                     (irValues[2] || irValues[3] || irValues[4]),
                     (lastOpponentState == 3) ? 1 : 0);
        break;
      }
      
      case 4: { // Opponent at right - more aggressive turn
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          motorCommands.leftSpeed = ATTACK_SPEED_FWD + 15;  // Faster forward
          motorCommands.rightSpeed = SEARCH_SPEED_BWD - 10; // Faster backward
          xSemaphoreGive(motorMutex);
        }
        
        logStateChange("Attacking right", irValues[5], (lastOpponentState == 4) ? 1 : 0);
        break;
      }
    }
    
    // Update state data for next iteration
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      stateData.lastOpponentState = opponentState;
      stateData.opponentState = opponentState;
      stateData.searchPatternState = searchPatternState;
      stateData.searchPatternTimer = searchPatternTimer;
      stateData.backAttackState = backAttackState;
      stateData.backAttackTimer = backAttackTimer;
      stateData.stateEntryTime = stateEntryTime;
      xSemaphoreGive(stateMutex);
    }
    
    // Delay until next decision update
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DECISION_UPDATE_DELAY));
  }
}

// Task to monitor the start button
void ButtonTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool buttonState = false;
  bool lastButtonState = false;
  
  for (;;) {
    buttonState = digitalRead(startPin);
    
    if (buttonState && !lastButtonState) {
      // Button pressed - start the robot
      if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        stateData.robotStarted = true;
        Serial.println("Robot started!");
        xSemaphoreGive(stateMutex);
      }
    }
    
    lastButtonState = buttonState;
    
    // Delay until next button check
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BUTTON_READ_DELAY));
  }
}

// Configure PWM on a pin
void setupPWM(uint pin, uint &slice) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num(pin);

  float divider = (float)125000000 / (PWM_FREQ * (PWM_RESOLUTION + 1));
  divider = constrain(divider, 1.0f, 255.0f);
  pwm_set_clkdiv(slice, divider);
  pwm_set_wrap(slice, PWM_RESOLUTION);
  pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
  pwm_set_enabled(slice, true);
}

// Set left motor speed (0-255)
void setLeftMotor(int speed) {
  digitalWrite(EN_left, HIGH);
  pwm_set_chan_level(slice_left, pwm_gpio_to_channel(PWM_left), speed);
}

// Set right motor speed (0-255)
void setRightMotor(int speed) {
  digitalWrite(EN_right, HIGH);
  pwm_set_chan_level(slice_right, pwm_gpio_to_channel(PWM_right), speed);
}

// Stop both motors
void stopMotors() {
  digitalWrite(EN_left, LOW);
  digitalWrite(EN_right, LOW);
  Serial.println("Motors stopped.");
}

// Static variables for logging
static unsigned long lastSerialOutput = 0;

// Log a message (limited by SERIAL_INTERVAL)
void logMessage(const char *message) {
  if (millis() - lastSerialOutput > SERIAL_INTERVAL) {
    Serial.println(message);
    lastSerialOutput = millis();
  }
}

// Log a message only when state changes
void logStateChange(const char *message, int newState, int oldState) {
  if (newState != oldState && millis() - lastSerialOutput > SERIAL_INTERVAL) {
    Serial.println(message);
    lastSerialOutput = millis();
  }
}

// Add the temperature mapping function
float mapTemperature(int analogValue) {
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

  return map(celsius, 50, -50, 24.0, 120.0); // Map to 0-1 range
}

// Add a function to print temperature values if needed
void printTemperatureInfo() {
  float temp1, temp2;
  
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    temp1 = sensorData.motorTemp1;
    temp2 = sensorData.motorTemp2;
    xSemaphoreGive(sensorMutex);
  }
  
  Serial.print("Motor temps: ");
  Serial.print(temp1);
  Serial.print("°C, ");
  Serial.print(temp2);
  Serial.println("°C");
}