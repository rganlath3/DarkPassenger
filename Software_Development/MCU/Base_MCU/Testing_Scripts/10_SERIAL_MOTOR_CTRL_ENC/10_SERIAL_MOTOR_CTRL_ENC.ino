/*
  By Ranil Ganlath
  This is for testing the Roomba Encoders with a 3V Power Source. With the ESP32, any GPIO pin can be configured as the digital input pullup pin.
  Board: ESP32-S3-WROOM-1
  ESP32 GND - Motor Driver GND
  ESP32 11  - Motor Driver PWM 1 Input
  ESP32 13  - Motor Driver DIR 1 Input
  ESP32 21  - Motor Driver PWM 2 Input
  ESP32 19  - Motor Driver DIR 2 Input


  ESP32 41  - Motor Encoder Left Input (internal pullup)
  ESP32 40  - Wheel Drop Switch Left Input
  ESP32 6  - Motor Encoder Right Input (internal pullup)
  ESP32 5  - Wheel Drop Switch Right Input

  IDE Configuration:
  Arduino Board: ESP32S3 Dev Module
  Upload Speed: 921600
  USB Mode: Hardware CDC and JTAG
  USB CDC On Boot: Disabled
  USB Firmware MSC On Boot: Disabled
  USB DFU On Boot: Disabled
  Upload Mode: UART0/Hardware CDC
  CPU Freq: 240MHz
  Flash Mode: QIO 80MHz
  Flash Size: 4MB
  Parition Scheme: Default 4MB with spiffs
  Core Debug Level: None
  PSRAM: Disabled
  Arduino Runs On: Core 1
  Events Run On: Core 1
  Erash All Flash Before Sketch Upload: Disabled
  JTAG Adapter: Disabled
  Zigbee Mode: Disabled
*/

#include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 11, 13);  // PWM 1 = Pin 11, DIR 1 = Pin 13.
CytronMD motor2(PWM_DIR, 21, 19);  // PWM 2 = Pin 21, DIR 2 = Pin 19.

// Pin definitions
#define LEFT_ENC_PIN 41
#define RIGHT_ENC_PIN 6
#define LEFT_WHEEL_SW_PIN 40
#define RIGHT_WHEEL_SW_PIN 5

// Motor speed constants
#define MOTOR_SPEED 64      // Standard movement speed (25% of max)
#define TURNING_SPEED 64    // Speed for turns

// Encoder parameters
#define TRANSITIONS_PER_REV 508.8  // 8 transitions × 63.6:1 gear ratio
#define WHEEL_DIAMETER_MM 70.0     // Roomba wheel diameter in mm (adjust if different)
#define PI 3.14159265359
#define MM_TO_INCHES 0.0393701     // Conversion factor from mm to inches

// Encoder tracking variables
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
int lastLeftState = 0;
int lastRightState = 0;
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // Update every 100ms

char command = 'S';         // Default to Stop

// Distance tracking
float leftDistanceMM = 0.0;
float rightDistanceMM = 0.0;

// Function to calculate distance from encoder counts
float calculateDistance(unsigned long counts) {
  float revolutions = counts / TRANSITIONS_PER_REV;
  return revolutions * PI * WHEEL_DIAMETER_MM;
}

// The setup routine runs once when you press reset.
void setup() {
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  pinMode(LEFT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_SW_PIN, INPUT_PULLUP);
  
  Serial.begin(115200);
  while(!Serial);
  
  // Initialize last encoder states
  lastLeftState = digitalRead(LEFT_ENC_PIN);
  lastRightState = digitalRead(RIGHT_ENC_PIN);
  
  Serial.println("Roomba Serial Controller Started with Encoder Distance Tracking");
  Serial.println("Commands: F (forward), B (backward), R (right), L (left), S (stop), C (clear counters)");
  Serial.println("Encoder resolution: 508.8 transitions per wheel revolution");
  
  // Initially stopped
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

// The loop routine runs over and over again forever.
void loop() {
  // Read encoder states and count transitions
  int currentLeftState = digitalRead(LEFT_ENC_PIN);
  int currentRightState = digitalRead(RIGHT_ENC_PIN);
  
  // Check for left encoder transition
  if (currentLeftState != lastLeftState) {
    leftEncoderCount++;
    lastLeftState = currentLeftState;
  }
  
  // Check for right encoder transition
  if (currentRightState != lastRightState) {
    rightEncoderCount++;
    lastRightState = currentRightState;
  }
  
  // Check for serial commands
  if (Serial.available() > 0) {
    command = Serial.read();
    
    // Convert lowercase to uppercase for convenience
    if (command >= 'a' && command <= 'z') {
      command = command - 'a' + 'A';
    }
    
    // Process the command
    switch (command) {
      case 'F':  // Forward
        Serial.println("Moving Forward");
        motor1.setSpeed(-MOTOR_SPEED);   // Left motor forward
        motor2.setSpeed(-MOTOR_SPEED);   // Right motor forward
        break;
        
      case 'B':  // Backward
        Serial.println("Moving Backward");
        motor1.setSpeed(MOTOR_SPEED);  // Left motor backward
        motor2.setSpeed(MOTOR_SPEED);  // Right motor backward
        break;
        
      case 'R':  // Right turn
        Serial.println("Turning Right");
        motor1.setSpeed(-TURNING_SPEED);  // Left motor forward
        motor2.setSpeed(TURNING_SPEED);   // Right motor backward
        break;
        
      case 'L':  // Left turn
        Serial.println("Turning Left");
        motor1.setSpeed(TURNING_SPEED);    // Left motor backward
        motor2.setSpeed(-TURNING_SPEED);   // Right motor forward
        break;
        
      case 'S':  // Stop
        Serial.println("Stopping");
        motor1.setSpeed(0);  // Stop left motor
        motor2.setSpeed(0);  // Stop right motor
        break;
        
      case 'C':  // Clear distance counters
        Serial.println("Clearing encoder counts and distance");
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        leftDistanceMM = 0;
        rightDistanceMM = 0;
        break;
        
      default:
        // Do nothing for unrecognized commands
        break;
    }
  }
  
  // Update and display info periodically
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    // Calculate distances
    leftDistanceMM = calculateDistance(leftEncoderCount);
    rightDistanceMM = calculateDistance(rightEncoderCount);
    
    // Calculate average distance
    float avgDistanceMM = (leftDistanceMM + rightDistanceMM) / 2;
    
    // Convert to inches
    float leftDistanceInches = leftDistanceMM * MM_TO_INCHES;
    float rightDistanceInches = rightDistanceMM * MM_TO_INCHES;
    float avgDistanceInches = avgDistanceMM * MM_TO_INCHES;
    
    // Print sensor and encoder data
    Serial.println("-------- Sensor Data --------");
    Serial.println("Left Wheel Switch: " + String(digitalRead(LEFT_WHEEL_SW_PIN)) + 
                  ", Right Wheel Switch: " + String(digitalRead(RIGHT_WHEEL_SW_PIN)));
    Serial.println("Left Encoder State: " + String(currentLeftState) + 
                  ", Right Encoder State: " + String(currentRightState));
    
    Serial.println("-------- Distance Data --------");
    Serial.println("Left Encoder Count: " + String(leftEncoderCount) + 
                  ", Right Encoder Count: " + String(rightEncoderCount));
                  
    // Print distances in mm
    Serial.println("Left Distance: " + String(leftDistanceMM, 1) + " mm (" + 
                  String(leftDistanceMM/1000, 2) + " m)");
    Serial.println("Right Distance: " + String(rightDistanceMM, 1) + " mm (" + 
                  String(rightDistanceMM/1000, 2) + " m)");
    Serial.println("Average Distance: " + String(avgDistanceMM, 1) + " mm (" + 
                  String(avgDistanceMM/1000, 2) + " m)");
                  
    // Print distances in inches
    Serial.println("Left Distance: " + String(leftDistanceInches, 1) + " in (" + 
                  String(leftDistanceInches/12, 2) + " ft)");
    Serial.println("Right Distance: " + String(rightDistanceInches, 1) + " in (" + 
                  String(rightDistanceInches/12, 2) + " ft)");
    Serial.println("Average Distance: " + String(avgDistanceInches, 1) + " in (" + 
                  String(avgDistanceInches/12, 2) + " ft)");
                  
    Serial.println();
    
    lastUpdateTime = currentTime;
  }
}
