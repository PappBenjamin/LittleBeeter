#include <stdio.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/irq.h"

// FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>

// Import our constants
#include "constants.h"

// Global objects
QTRSensors qtr;

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t decisionTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t edgeDetectionTaskHandle = NULL;

// Semaphores for protecting shared data
SemaphoreHandle_t sensorMutex = NULL;
SemaphoreHandle_t motorMutex = NULL;
SemaphoreHandle_t stateMutex = NULL;

// Events for interrupts and notifications
SemaphoreHandle_t edgeDetectedSemaphore = NULL;
QueueHandle_t sensorDataQueue = NULL;
QueueHandle_t motorCommandQueue = NULL;

// PWM configuration variables
uint slice_left;
uint slice_right;

// Shared sensor data (protected by sensorMutex)
struct SensorData {
    uint16_t qtrValues[SensorCount];
    int irValues[IRCount];
    bool edgeDetected;
    float motorTemp1;
    float motorTemp2;
    int irConfirmedCount[IRCount];
    TickType_t timestamp;  // FreeRTOS tick timestamp
};

// Shared motor commands (protected by motorMutex)
struct MotorCommands {
    int leftSpeed;
    int rightSpeed;
    TickType_t timestamp;  // FreeRTOS tick timestamp
};

// Shared state data (protected by stateMutex)
struct StateData {
    int opponentState;
    int lastOpponentState;
    int searchPatternState;
    TickType_t searchPatternTimer;
    bool robotStarted;
    int backAttackState;
    TickType_t backAttackTimer;
    TickType_t stateEntryTime;
    int opponentTrackingCount[IRCount];  // Track consecutive detections per sensor
};

// Global instances of shared data
SensorData sensorData;
MotorCommands motorCommands;
StateData stateData;

// Function prototypes
void SensorTask(void *pvParameters);
void MotorControlTask(void *pvParameters);
void DecisionTask(void *pvParameters);
void ButtonTask(void *pvParameters);
void EdgeDetectionTask(void *pvParameters);
void setupPWM(uint pin, uint &slice);
void setLeftMotor(int speed);
void setRightMotor(int speed);
void stopMotors();
void logMessage(const char *message);
void logStateChange(const char *message, int newState, int oldState);
float mapTemperature(int analogValue);
void printTemperatureInfo();
void configureQTRInterrupts();
void qtrInterruptHandler();

// Setup function
void setup() {
    // Initialize serial communication
    Serial.begin(115200);  // Higher baud rate for better performance
    // Don't wait for serial - this is a robot
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
    pinMode(startPin, INPUT_PULLDOWN);  // Add pulldown to prevent floating input

    // Configure PWM for motors
    setupPWM(PWM_left, slice_left);
    setupPWM(PWM_right, slice_right);
    
    // Configure temperature sensors
    pinMode(TMP_1, INPUT);
    pinMode(TMP_2, INPUT);
    
    // Initialize shared data structures
    sensorData.edgeDetected = false;
    sensorData.timestamp = 0;
    
    for (int i = 0; i < SensorCount; i++) {
        sensorData.qtrValues[i] = 0;
    }
    
    for (int i = 0; i < IRCount; i++) {
        sensorData.irValues[i] = 0;
        sensorData.irConfirmedCount[i] = 0;
    }
    
    motorCommands.leftSpeed = STOP_SPEED;
    motorCommands.rightSpeed = STOP_SPEED;
    motorCommands.timestamp = 0;
    
    stateData.opponentState = 0;
    stateData.lastOpponentState = -1;
    stateData.searchPatternState = 0;
    stateData.searchPatternTimer = 0;
    stateData.robotStarted = false;
    stateData.backAttackState = 0;
    stateData.backAttackTimer = 0;
    stateData.stateEntryTime = 0;
    
    for (int i = 0; i < IRCount; i++) {
        stateData.opponentTrackingCount[i] = 0;
    }

    // Create FreeRTOS synchronization objects
    sensorMutex = xSemaphoreCreateMutex();
    motorMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    edgeDetectedSemaphore = xSemaphoreCreateBinary();
    
    // Create message queues
    sensorDataQueue = xQueueCreate(3, sizeof(SensorData));
    motorCommandQueue = xQueueCreate(3, sizeof(MotorCommands));
    
    // Ensure synchronization objects were created successfully
    if (sensorMutex == NULL || motorMutex == NULL || stateMutex == NULL || 
        edgeDetectedSemaphore == NULL || sensorDataQueue == NULL || motorCommandQueue == NULL) {
        Serial.println("ERROR: Failed to create one or more synchronization objects");
        while (1); // Hang if creation failed
    }

    // Set up QTR interrupts
    configureQTRInterrupts();

    // Create tasks with appropriate priorities
    BaseType_t xReturned;
    
    // Edge detection task - highest priority for quick response
    xReturned = xTaskCreate(
        EdgeDetectionTask,
        "EdgeTask",
        STACK_SIZE_SENSOR,
        NULL,
        PRIORITY_SENSOR_TASK + 2,  // Higher than sensor task
        &edgeDetectionTaskHandle
    );
    if (xReturned != pdPASS) {
        Serial.println("ERROR: Failed to create edge detection task");
    }
    
    // Sensor task - high priority for timely data acquisition
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
    
    // Motor control task - high priority for responsive movement
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
    
    // Decision making task - medium priority
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
    
    // Button monitoring task - lowest priority
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

// Configure QTR sensor interrupts
void configureQTRInterrupts() {
    // Note: This is a placeholder - actual implementation would depend on
    // the specific hardware capabilities of the RP2040 and how QTR sensors are connected
    // In a real implementation, you would configure GPIO interrupts to trigger on edge detection
    
    // Example (conceptual):
    // for (int i = 0; i < SensorCount; i++) {
    //     gpio_set_irq(QTRPins[i], GPIO_IRQ_LEVEL_LOW, &qtrInterruptHandler);
    // }
}

// QTR interrupt handler - would be called when a QTR sensor detects an edge
void qtrInterruptHandler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Signal the edge detection task
    xSemaphoreGiveFromISR(edgeDetectedSemaphore, &xHigherPriorityTaskWoken);
    
    // Yield to higher priority task if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task specifically for handling edge detection with high priority
void EdgeDetectionTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        // Wait for edge detection semaphore or perform regular polling
        if (xSemaphoreTake(edgeDetectedSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Edge detected via interrupt
            // Fall through to handle it
        } else {
            // No interrupt - poll QTR sensors directly for edge detection
            uint16_t qtrValues[SensorCount];
            qtr.read(qtrValues);
            
            bool edgeDetected = false;
            for (int i = 0; i < SensorCount; i++) {
                if (qtrValues[i] < EDGE_THRESHOLD) {
                    edgeDetected = true;
                    break;
                }
            }
            
            if (!edgeDetected) {
                // No edge detected, continue monitoring
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); // Fast polling rate
                continue;
            }
        }
        
        // Edge detected - take immediate action
        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Emergency stop first to prevent falling off
            motorCommands.leftSpeed = STOP_SPEED;
            motorCommands.rightSpeed = STOP_SPEED;
            xSemaphoreGive(motorMutex);
        }
        
        // Update edge detection status
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            sensorData.edgeDetected = true;
            xSemaphoreGive(sensorMutex);
        }
        
        // Notify the decision task about edge detection
        xTaskNotify(decisionTaskHandle, 0x01, eSetBits);
        
        // Brief pause to allow decision task to handle the edge
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Task to read sensors and update shared sensor data
void SensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t lastTempCheckTime = 0;
    
    for (;;) {
        // Get current time in ticks
        TickType_t currentTicks = xTaskGetTickCount();
        
        // Create local sensor data object
        SensorData localSensorData;
        localSensorData.timestamp = currentTicks;
        
        // Read QTR sensors if not handled by edge detection task
        qtr.read(localSensorData.qtrValues);
        
        // Read IR sensors with debouncing
        for (int i = 0; i < IRCount; i++) {
            int rawValue = digitalRead(IRPins[i]);
            
            // Apply debouncing logic
            if (rawValue == 1) {
                if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    sensorData.irConfirmedCount[i]++;
                    if (sensorData.irConfirmedCount[i] >= IR_DEBOUNCE_COUNT) {
                        localSensorData.irValues[i] = 1;
                    } else {
                        localSensorData.irValues[i] = 0;
                    }
                    // Copy to local data
                    localSensorData.irConfirmedCount[i] = sensorData.irConfirmedCount[i];
                    xSemaphoreGive(sensorMutex);
                }
            } else {
                if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    sensorData.irConfirmedCount[i] = 0;
                    localSensorData.irValues[i] = 0;
                    localSensorData.irConfirmedCount[i] = 0;
                    xSemaphoreGive(sensorMutex);
                }
            }
        }
        
        // Temperature safety check - runs every TEMP_CHECK_INTERVAL
        if ((currentTicks - lastTempCheckTime) >= pdMS_TO_TICKS(TEMP_CHECK_INTERVAL)) {
            lastTempCheckTime = currentTicks;
            
            // Read temperature sensors
            int tempValue1 = analogRead(TMP_1);
            int tempValue2 = analogRead(TMP_2);
            localSensorData.motorTemp1 = mapTemperature(tempValue1);
            localSensorData.motorTemp2 = mapTemperature(tempValue2);
            
            // Safety cutoff - if either temperature exceeds threshold
            if (localSensorData.motorTemp1 > MAX_TEMP_THRESHOLD || 
                localSensorData.motorTemp2 > MAX_TEMP_THRESHOLD) {
                
                // Emergency stop via motor commands
                MotorCommands emergencyStop;
                emergencyStop.leftSpeed = STOP_SPEED;
                emergencyStop.rightSpeed = STOP_SPEED;
                emergencyStop.timestamp = currentTicks;
                
                // Send emergency stop command with high priority
                xQueueSendToFront(motorCommandQueue, &emergencyStop, 0);
                
                // Notify all tasks about the emergency
                xTaskNotify(decisionTaskHandle, 0x02, eSetBits);  // Emergency bit
                xTaskNotify(motorTaskHandle, 0x02, eSetBits);     // Emergency bit
                
                // Log the emergency stop
                Serial.println("!!! EMERGENCY STOP - MOTOR OVERHEATING !!!");
                Serial.print("Temps: ");
                Serial.print(localSensorData.motorTemp1);
                Serial.print("°C, ");
                Serial.print(localSensorData.motorTemp2);
                Serial.println("°C");
            }
        } else {
            // Use cached temperature values
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                localSensorData.motorTemp1 = sensorData.motorTemp1;
                localSensorData.motorTemp2 = sensorData.motorTemp2;
                xSemaphoreGive(sensorMutex);
            }
        }
        
        // Determine if edge detected from QTR values
        localSensorData.edgeDetected = false;
        for (int i = 0; i < SensorCount; i++) {
            if (localSensorData.qtrValues[i] < EDGE_THRESHOLD) {
                localSensorData.edgeDetected = true;
                break;
            }
        }
        
        // Edge detection takes priority - notify immediately if detected
        if (localSensorData.edgeDetected) {
            // Signal the edge detection semaphore
            xSemaphoreGive(edgeDetectedSemaphore);
        }
        
        // Update shared sensor data with mutex protection
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Update the global sensor data
            memcpy(&sensorData, &localSensorData, sizeof(SensorData));
            xSemaphoreGive(sensorMutex);
        }
        
        // Send sensor data to queue for decision task
        xQueueSend(sensorDataQueue, &localSensorData, 0);
        
        // Delay until next sensor reading
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_READ_DELAY));
    }
}

// Task to control motors based on motor commands
void MotorControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int leftSpeed = STOP_SPEED;
    int rightSpeed = STOP_SPEED;
    uint32_t notificationValue;
    
    for (;;) {
        // Check for emergency notifications
        if (xTaskNotifyWait(0, 0x02, &notificationValue, 0) == pdTRUE) {
            if (notificationValue & 0x02) {
                // Emergency condition - stop motors immediately
                stopMotors();
                vTaskDelay(pdMS_TO_TICKS(100));  // Short delay during emergency
                continue;
            }
        }
        
        // Check if there are new motor commands in the queue
        MotorCommands newCommands;
        if (xQueueReceive(motorCommandQueue, &newCommands, 0) == pdTRUE) {
            // New commands received
            leftSpeed = newCommands.leftSpeed;
            rightSpeed = newCommands.rightSpeed;
        } else {
            // No new commands in queue, check shared memory
            if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                leftSpeed = motorCommands.leftSpeed;
                rightSpeed = motorCommands.rightSpeed;
                xSemaphoreGive(motorMutex);
            }
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
    SensorData localSensorData;
    int irValues[IRCount];
    int opponentState = 0;
    int lastOpponentState = -1;
    int searchPatternState = 0;
    TickType_t searchPatternTimer = 0;
    int backAttackState = 0;
    TickType_t backAttackTimer = 0;
    TickType_t stateEntryTime = 0;
    uint32_t notificationValue;
    
    // Strategy enhancers
    int opponentTrackingCount[IRCount] = {0};
    bool edgeAvoidanceActive = false;
    TickType_t edgeAvoidanceTimer = 0;
    
    for (;;) {
        TickType_t currentTicks = xTaskGetTickCount();
        
        // Check for notification flags
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, 0) == pdTRUE) {
            // Edge detection notification (0x01)
            if (notificationValue & 0x01) {
                edgeAvoidanceActive = true;
                edgeAvoidanceTimer = currentTicks;
                
                // Response to edge detection is immediate
                MotorCommands edgeResponse;
                edgeResponse.leftSpeed = RETREAT_SPEED_BWD - 5;  // Faster retreat
                edgeResponse.rightSpeed = RETREAT_SPEED_BWD - 5;
                edgeResponse.timestamp = currentTicks;
                
                // Send to motor control
                xQueueSendToFront(motorCommandQueue, &edgeResponse, 0);
                
                // Log the edge detection
                logMessage("EDGE DETECTED - RETREATING");
            }
            
            // Emergency notification (0x02)
            if (notificationValue & 0x02) {
                // Emergency condition - reset strategy
                searchPatternState = 0;
                backAttackState = 0;
                edgeAvoidanceActive = false;
                
                // Log the emergency
                logMessage("EMERGENCY - RESET STRATEGY");
                
                // If emergency is severe, wait a moment
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        // First check if robot should be active
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            robotActive = stateData.robotStarted;
            xSemaphoreGive(stateMutex);
        }
        
        if (!robotActive) {
            // Robot not started yet, skip decision logic
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DECISION_UPDATE_DELAY));
            continue;
        }
        
        // Handle edge avoidance maneuver if active
        if (edgeAvoidanceActive) {
            // First phase: retreat for 200ms
            if (currentTicks - edgeAvoidanceTimer < pdMS_TO_TICKS(200)) {
                // Continue retreating (already issued commands)
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
                continue;
            } 
            // Second phase: turn for 150ms
            else if (currentTicks - edgeAvoidanceTimer < pdMS_TO_TICKS(350)) {
                // Turn away from edge
                MotorCommands turnCommand;
                turnCommand.leftSpeed = RETREAT_SPEED_BWD - 10;
                turnCommand.rightSpeed = RETREAT_SPEED_FWD + 10;
                turnCommand.timestamp = currentTicks;
                
                xQueueSend(motorCommandQueue, &turnCommand, 0);
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
                continue;
            } 
            // Completed edge avoidance maneuver
            else {
                edgeAvoidanceActive = false;
                // Fall through to normal decision making
            }
        }
        
        // Get latest sensor data - prefer from queue for newest
        if (xQueueReceive(sensorDataQueue, &localSensorData, 0) != pdTRUE) {
            // No data in queue, use shared memory
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(&localSensorData, &sensorData, sizeof(SensorData));
                xSemaphoreGive(sensorMutex);
            }
        }
        
        // Copy IR values for processing
        for (int i = 0; i < IRCount; i++) {
            irValues[i] = localSensorData.irValues[i];
        }
        
        // Check for new edge detection
        if (localSensorData.edgeDetected && !edgeAvoidanceActive) {
            edgeAvoidanceActive = true;
            edgeAvoidanceTimer = currentTicks;
            
            // Response to edge detection
            MotorCommands edgeResponse;
            edgeResponse.leftSpeed = RETREAT_SPEED_BWD - 5;
            edgeResponse.rightSpeed = RETREAT_SPEED_BWD - 5;
            edgeResponse.timestamp = currentTicks;
            
            // Send to motor control with high priority
            xQueueSendToFront(motorCommandQueue, &edgeResponse, 0);
            
            // Skip other decision making this cycle
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
            continue;
        }
        
        // Get state information from shared memory
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lastOpponentState = stateData.lastOpponentState;
            searchPatternState = stateData.searchPatternState;
            searchPatternTimer = stateData.searchPatternTimer;
            backAttackState = stateData.backAttackState;
            backAttackTimer = stateData.backAttackTimer;
            stateEntryTime = stateData.stateEntryTime;
            
            // Copy opponent tracking data
            for (int i = 0; i < IRCount; i++) {
                opponentTrackingCount[i] = stateData.opponentTrackingCount[i];
            }
            
            xSemaphoreGive(stateMutex);
        }
        
        // Update opponent tracking counts for each IR sensor
        for (int i = 0; i < IRCount; i++) {
            if (irValues[i] == 1) {
                if (opponentTrackingCount[i] < 10) {  // Cap at 10 for stability
                    opponentTrackingCount[i]++;
                }
            } else {
                if (opponentTrackingCount[i] > 0) {
                    opponentTrackingCount[i]--;
                }
            }
        }
        
        // Enhanced opponent state determination using tracking counts
        opponentState = 0;
        
        // Check for opponent with preference given to higher count sensors
        int maxCount = 0;
        int maxSensor = -1;
        
        for (int i = 0; i < IRCount; i++) {
            if (opponentTrackingCount[i] > maxCount) {
                maxCount = opponentTrackingCount[i];
                maxSensor = i;
            }
        }
        
        // Require minimum count to consider opponent detected
        if (maxCount >= 2) {
            // Map sensor index to opponent position
            switch (maxSensor) {
                case 0: opponentState = 1; break;  // Back
                case 1: opponentState = 2; break;  // Left
                case 2: // Front-left
                case 3: // Center
                case 4: opponentState = 3; break;  // Front-right
                case 5: opponentState = 4; break;  // Right
            }
            
            // Special case for front center (more precise detection)
            if (maxSensor == 2 || maxSensor == 3 || maxSensor == 4) {
                // Check if opponent is directly in front or off-center
                if (opponentTrackingCount[2] > opponentTrackingCount[4]) {
                    // More to the left
                    opponentState = 3;  // Center-left
                } else if (opponentTrackingCount[4] > opponentTrackingCount[2]) {
                    // More to the right
                    opponentState = 3;  // Center-right
                } else {
                    // Directly center
                    opponentState = 3;  // Direct center
                }
            }
        }
        
        // Check if stuck in a state too long - reset if needed
        if ((currentTicks - stateEntryTime) > pdMS_TO_TICKS(MAX_STATE_TIME)) {
            // Force a state change if stuck too long
            backAttackState = 0;
            searchPatternState = 0;
            stateEntryTime = currentTicks;
            logMessage("State timeout - resetting strategy");
        }
        
        // Reset state entry timer if state changed
        if (opponentState != lastOpponentState) {
            stateEntryTime = currentTicks;
        }
        
        // Create command for motor control
        MotorCommands command;
        command.timestamp = currentTicks;
        
        // 3. React based on opponent position
        switch (opponentState) {
            case 0: { // No opponent - search pattern
                // Reset search state if we just lost track of opponent
                if (lastOpponentState != 0) {
                    searchPatternState = 0;
                    searchPatternTimer = currentTicks;
                    logMessage("Starting search pattern");
                }
                
                // State machine for search pattern
                if (searchPatternState == 0) { // Forward phase
                    command.leftSpeed = SEARCH_SPEED_FWD + 5;  // Slightly faster
                    command.rightSpeed = SEARCH_SPEED_FWD + 3; // Slight curve to search more area
                    
                    if ((currentTicks - searchPatternTimer) >= pdMS_TO_TICKS(FORWARD_TIME)) {
                        searchPatternState = 1;
                        searchPatternTimer = currentTicks;
                    }
                }
                else if (searchPatternState == 1) { // Backward phase
                    command.leftSpeed = SEARCH_SPEED_BWD - 3; // Slightly faster
                    command.rightSpeed = SEARCH_SPEED_BWD;    // Slight curve for better coverage
                    
                    if ((currentTicks - searchPatternTimer) >= pdMS_TO_TICKS(BACKWARD_TIME)) {
                        searchPatternState = 2;
                        searchPatternTimer = currentTicks;
                    }
                }
                else if (searchPatternState == 2) { // Turn phase - wider turn
                    command.leftSpeed = SEARCH_SPEED_FWD + 10;  // Faster spin
                    command.rightSpeed = SEARCH_SPEED_BWD - 10; // Faster spin
                    
                    if ((currentTicks - searchPatternTimer) >= pdMS_TO_TICKS(TURN_TIME + 10)) { // Slightly longer turn
                        searchPatternState = 0;
                        searchPatternTimer = currentTicks;
                    }
                }
                break;
            }
            
            case 1: { // Opponent at back - improved state machine
                // If we just detected opponent at back, initialize turn
                if (lastOpponentState != 1) {
                    backAttackState = 0;
                    backAttackTimer = currentTicks;
                    
                    // Start with brief reverse to gain space
                    command.leftSpeed = SEARCH_SPEED_BWD - 5;
                    command.rightSpeed = SEARCH_SPEED_BWD - 5;
                    
                    logMessage("Detected opponent at back - preparing 180° turn");
                }
                else {
                    // State machine for back opponent handling
                    if (backAttackState == 0 && 
                        (currentTicks - backAttackTimer) >= pdMS_TO_TICKS(BACK_ATTACK_REVERSE_TIME)) {
                        // After brief reverse, start fast 180° turn
                        command.leftSpeed = SEARCH_SPEED_FWD + 15;  // Faster turn
                        command.rightSpeed = SEARCH_SPEED_BWD - 15; // Faster turn
                        backAttackState = 1;
                        backAttackTimer = currentTicks;
                        logMessage("Starting 180° turn");
                    }
                    else if (backAttackState == 0) {
                        // Still in initial reverse
                        command.leftSpeed = SEARCH_SPEED_BWD - 5;
                        command.rightSpeed = SEARCH_SPEED_BWD - 5;
                    }
                    else if (backAttackState == 1 && 
                            (currentTicks - backAttackTimer) >= pdMS_TO_TICKS(BACK_ATTACK_TURN_TIME)) {
                        // Turn complete - attack at full speed
                        command.leftSpeed = ATTACK_SPEED_FWD + 20;
                        command.rightSpeed = ATTACK_SPEED_FWD + 20;
                        backAttackState = 2;
                        logMessage("Turn complete - attacking");
                    }
                    else if (backAttackState == 1) {
                        // Still turning
                        command.leftSpeed = SEARCH_SPEED_FWD + 15;
                        command.rightSpeed = SEARCH_SPEED_BWD - 15;
                    }
                    else if (backAttackState == 2) {
                        // Continue attack
                        command.leftSpeed = ATTACK_SPEED_FWD + 20;
                        command.rightSpeed = ATTACK_SPEED_FWD + 20;
                    }
                }
                break;
            }
            
            case 2: { // Opponent at left - more aggressive turn
                command.leftSpeed = SEARCH_SPEED_BWD - 10;  // Faster backward
                command.rightSpeed = ATTACK_SPEED_FWD + 15; // Faster forward
                
                logStateChange("Attacking left", 
                               opponentTrackingCount[1], 
                               (lastOpponentState == 2) ? opponentTrackingCount[1] : 0);
                break;
            }
            
            case 3: { // Opponent at center - improved directional attack
                if (opponentTrackingCount[2] > opponentTrackingCount[4]) {
                    // Opponent slightly left of center
                    command.leftSpeed = ATTACK_SPEED_FWD + 10;
                    command.rightSpeed = ATTACK_SPEED_FWD + 30; // More aggressive turn
                } else if (opponentTrackingCount[4] > opponentTrackingCount[2]) {
                    // Opponent slightly right of center
                    command.leftSpeed = ATTACK_SPEED_FWD + 30; // More aggressive turn
                    command.rightSpeed = ATTACK_SPEED_FWD + 10;
                } else {
                    // Direct center attack with max speed
                    command.leftSpeed = ATTACK_SPEED_FWD + 40; // Maximum attack speed
                    command.rightSpeed = ATTACK_SPEED_FWD + 40;
                }
                
                logStateChange("Attacking center",
                             (opponentTrackingCount[2] + opponentTrackingCount[3] + opponentTrackingCount[4]),
                             (lastOpponentState == 3) ? (opponentTrackingCount[2] + opponentTrackingCount[3] + opponentTrackingCount[4]) : 0);
                break;
            }
            
            case 4: { // Opponent at right - more aggressive turn
                command.leftSpeed = ATTACK_SPEED_FWD + 15;  // Faster forward
                command.rightSpeed = SEARCH_SPEED_BWD - 10; // Faster backward
                
                logStateChange("Attacking right", 
                               opponentTrackingCount[5], 
                               (lastOpponentState == 4) ? opponentTrackingCount[5] : 0);
                break;
            }
        }
        
        // Send motor command to queue
        xQueueSend(motorCommandQueue, &command, 0);
        
        // Also update shared motor commands for backup
        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            motorCommands.leftSpeed = command.leftSpeed;
            motorCommands.rightSpeed = command.rightSpeed;
            motorCommands.timestamp = command.timestamp;
            xSemaphoreGive(motorMutex);
        }
        
        // Update state data for next iteration
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            stateData.lastOpponentState = opponentState;
            stateData.opponentState = opponentState;
            stateData.searchPatternState = searchPatternState;
            stateData.searchPatternTimer = searchPatternTimer;
            stateData.backAttackState = backAttackState;
            stateData.backAttackTimer = backAttackTimer;
            stateData.stateEntryTime = stateEntryTime;
            
            // Update opponent tracking data
            for (int i = 0; i < IRCount; i++) {
                stateData.opponentTrackingCount[i] = opponentTrackingCount[i];
            }
            
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
                xSemaphoreGive(stateMutex);
            }
            
            // Notify all tasks that robot has started
            xTaskNotify(decisionTaskHandle, 0x04, eSetBits);  // Start bit
            
            // Log the start event
            Serial.println("Robot started!");
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
    if ((xTaskGetTickCount() - lastSerialOutput) > pdMS_TO_TICKS(SERIAL_INTERVAL)) {
        Serial.println(message);
        lastSerialOutput = xTaskGetTickCount();
    }
}

// Log a message only when state changes
void logStateChange(const char *message, int newState, int oldState) {
    if (newState != oldState && 
        (xTaskGetTickCount() - lastSerialOutput) > pdMS_TO_TICKS(SERIAL_INTERVAL)) {
        Serial.println(message);
        lastSerialOutput = xTaskGetTickCount();
    }
}

// Temperature mapping function
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

// Function to print temperature values if needed
void printTemperatureInfo() {
    float temp1, temp2;
    
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        temp1 = sensorData.motorTemp1;
        temp2 = sensorData.motorTemp2;
        xSemaphoreGive(sensorMutex);
    
        Serial.print("Motor temps: ");
        Serial.print(temp1);
        Serial.print("°C, ");
        Serial.print(temp2);
        Serial.println("°C");
    }
}