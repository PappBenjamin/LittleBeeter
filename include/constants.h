#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

/*===== PIN DEFINITIONS =====*/
// Motor control pins
#define EN_left 10
#define EN_right 8
#define PWM_left 11
#define PWM_right 9
#define startPin 12 // starter pin

// IR sensor pins for opponent detection (6 sensors)
// Arrangement: back, left, front-left, center, front-right, right
const int IRPins[] = {13, 14, 6, 20, 7, 21};
const int IRCount = 6;

// QTR sensor pins for edge detection (4 sensors)
const uint8_t QTRPins[] = {2, 3, 4, 5};
const int SensorCount = 4;

/*===== MOVEMENT SETTINGS =====*/
// Motor speeds (0-255, 128 is stop)
#define STOP_SPEED 128
#define ATTACK_SPEED_FWD 160  // Forward attack
#define ATTACK_SPEED_BWD 96   // Backward attack
#define RETREAT_SPEED_FWD 140 // Forward retreat
#define RETREAT_SPEED_BWD 116 // Backward retreat
#define SEARCH_SPEED_FWD 140  // Forward search
#define SEARCH_SPEED_BWD 116  // Backward search

// Search pattern timing (in milliseconds)
const unsigned int FORWARD_TIME = 500;  // Forward phase
const unsigned int BACKWARD_TIME = 500; // Backward phase
const unsigned int TURN_TIME = 25;      // Turn phase

/*===== THRESHOLDS =====*/
#define EDGE_THRESHOLD 400    // Value indicating ring edge
#define OPPONENT_THRESHOLD 1  // Value indicating opponent detected

/*===== PWM CONFIGURATION =====*/
#define PWM_FREQ 3900
#define PWM_RESOLUTION 255

/*===== FREERTOS CONFIGURATION =====*/
// Task priorities (higher number = higher priority)
#define PRIORITY_SENSOR_TASK 3
#define PRIORITY_MOTOR_TASK 4
#define PRIORITY_DECISION_TASK 2
#define PRIORITY_BUTTON_TASK 1

// Task stack sizes
#define STACK_SIZE_SENSOR 256
#define STACK_SIZE_MOTOR 256
#define STACK_SIZE_DECISION 256
#define STACK_SIZE_BUTTON 128

// Task delays in milliseconds
#define SENSOR_READ_DELAY 20      // 50Hz
#define MOTOR_UPDATE_DELAY 20     // 50Hz
#define DECISION_UPDATE_DELAY 50  // 20Hz
#define BUTTON_READ_DELAY 100     // 10Hz

// Serial output interval
#define SERIAL_INTERVAL 300       // milliseconds

#endif // CONSTANTS_H