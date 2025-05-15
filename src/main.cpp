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
  
  for (;;) {
    // Read QTR sensors
    uint16_t qtrValues[SensorCount];
    qtr.read(qtrValues);
    
    // Read IR sensors
    int irValues[IRCount];
    for (int i = 0; i < IRCount; i++) {
      irValues[i] = digitalRead(IRPins[i]);
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
        sensorData.irValues[i] = irValues[i];
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
  
  for (;;) {
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
      xSemaphoreGive(stateMutex);
    }
    
    // 1. Edge detection (highest priority)
    if (edgeDetected) {
      if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
        motorCommands.leftSpeed = RETREAT_SPEED_BWD;
        motorCommands.rightSpeed = RETREAT_SPEED_BWD;
        xSemaphoreGive(motorMutex);
      }
      
      // Wait a bit then turn away from edge
      vTaskDelay(pdMS_TO_TICKS(200));
      
      if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
        motorCommands.leftSpeed = RETREAT_SPEED_BWD;
        motorCommands.rightSpeed = RETREAT_SPEED_FWD;
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
    
    // 3. React based on opponent position
    switch (opponentState) {
      case 0: { // No opponent - search pattern
        unsigned long currentMillis = millis();
        
        // Reset search state if we just lost track of opponent
        if (lastOpponentState != 0) {
          searchPatternState = 0;
          searchPatternTimer = currentMillis;
        }
        
        // State machine for search pattern
        if (searchPatternState == 0) { // Forward phase
          if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            motorCommands.leftSpeed = SEARCH_SPEED_FWD;
            motorCommands.rightSpeed = SEARCH_SPEED_FWD - 1;
            xSemaphoreGive(motorMutex);
          }
          
          if (currentMillis - searchPatternTimer >= FORWARD_TIME) {
            searchPatternState = 1;
            searchPatternTimer = currentMillis;
          }
        }
        else if (searchPatternState == 1) { // Backward phase
          if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            motorCommands.leftSpeed = SEARCH_SPEED_BWD;
            motorCommands.rightSpeed = SEARCH_SPEED_BWD - 1;
            xSemaphoreGive(motorMutex);
          }
          
          if (currentMillis - searchPatternTimer >= BACKWARD_TIME) {
            searchPatternState = 2;
            searchPatternTimer = currentMillis;
          }
        }
        else if (searchPatternState == 2) { // Turn phase
          if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            motorCommands.leftSpeed = SEARCH_SPEED_FWD;
            motorCommands.rightSpeed = SEARCH_SPEED_BWD;
            xSemaphoreGive(motorMutex);
          }
          
          if (currentMillis - searchPatternTimer >= TURN_TIME) {
            searchPatternState = 0;
            searchPatternTimer = currentMillis;
          }
        }
        break;
      }
      
      case 1: { // Opponent at back
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          // Faster backward attack with slight direction bias based on last attack
          if (lastOpponentState == 2) { // If opponent was recently at left
            motorCommands.leftSpeed = ATTACK_SPEED_BWD - 10; // Turn more sharply
            motorCommands.rightSpeed = ATTACK_SPEED_BWD + 5;
          } else if (lastOpponentState == 4) { // If opponent was recently at right
            motorCommands.leftSpeed = ATTACK_SPEED_BWD + 5;
            motorCommands.rightSpeed = ATTACK_SPEED_BWD - 10; // Turn more sharply
          } else {
            // Faster backward attack
            motorCommands.leftSpeed = ATTACK_SPEED_BWD - 10; // More speed (lower value = faster backward)
            motorCommands.rightSpeed = ATTACK_SPEED_BWD - 10;
          }
          xSemaphoreGive(motorMutex);
        }
        
        logStateChange("Attacking backward", irValues[0], (lastOpponentState == 1) ? 1 : 0);
        break;
      }
      
      case 2: { // Opponent at left
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          // More aggressive left turn with initial burst
          motorCommands.leftSpeed = ATTACK_SPEED_BWD - 10; // Higher backward speed
          motorCommands.rightSpeed = ATTACK_SPEED_FWD + 15; // Higher forward speed
          xSemaphoreGive(motorMutex);
        }
        
        logStateChange("Attacking left", irValues[1], (lastOpponentState == 2) ? 1 : 0);
        break;
      }
      
      case 3: { // Opponent at center
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          // Optimize frontal attack with directional bias based on which sensors are active
          if (irValues[2] && !irValues[4]) {
            // Opponent slightly left of center - adjust trajectory
            motorCommands.leftSpeed = ATTACK_SPEED_FWD + 15;
            motorCommands.rightSpeed = ATTACK_SPEED_FWD + 25; // Right motor faster to turn slightly left
          } else if (!irValues[2] && irValues[4]) {
            // Opponent slightly right of center - adjust trajectory
            motorCommands.leftSpeed = ATTACK_SPEED_FWD + 25; // Left motor faster to turn slightly right
            motorCommands.rightSpeed = ATTACK_SPEED_FWD + 15;
          } else {
            // Direct center attack with max speed
            motorCommands.leftSpeed = ATTACK_SPEED_FWD + 30; // Much higher speed
            motorCommands.rightSpeed = ATTACK_SPEED_FWD + 30;
          }
          xSemaphoreGive(motorMutex);
        }
        
        logStateChange("Attacking center",
                     (irValues[2] || irValues[3] || irValues[4]),
                     (lastOpponentState == 3) ? 1 : 0);
        break;
      }
      
      case 4: { // Opponent at right
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
          // More aggressive right turn with initial burst
          motorCommands.leftSpeed = ATTACK_SPEED_FWD + 15; // Higher forward speed
          motorCommands.rightSpeed = ATTACK_SPEED_BWD - 10; // Higher backward speed
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