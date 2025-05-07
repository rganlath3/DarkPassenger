/*
  ESP32 PID Tuner for Motor Control
  Fixed for bidirectional operation
*/

// Pin definitions - only keeping motor and encoder related pins
#define MOTOR_DRIVER_PWM1_PIN 11
#define MOTOR_DRIVER_DIR1_PIN 13
#define MOTOR_DRIVER_PWM2_PIN 21
#define MOTOR_DRIVER_DIR2_PIN 19
#define LEFT_ENC_PIN 41
#define RIGHT_ENC_PIN 6

// Motor speed constants
#define MAX_MOTOR_SPEED 125  // Maximum PWM value (0-255)

// Encoder parameters
#define TRANSITIONS_PER_REV 508.8  // 8 transitions × 63.6:1 gear ratio
#define WHEEL_DIAMETER_MM 70.0     // Wheel diameter in mm
#define PI 3.14159265359

// Serial plotting parameters
#define SERIAL_BAUD_RATE 115200
#define PLOT_INTERVAL 50  // Update plot every 50ms

// Serial command handling
String inputString = "";      // String to hold incoming serial data
boolean stringComplete = false;  // Whether the string is complete

// Target profile
int TEST_MODE = 3;       // 1=sine wave, 2=step response, 3=ramp

// class for computing motor control signal
class SimplePID {
  private:
    float kp, kd, ki, umax; // Params for pid gains and max motor speed
    float eprev, eintegral; // Storage for derivative and integral from prior loop
  public:
    // Constructor
    SimplePID(): kp(1.0), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    
    // Function to set params
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }
    
    // Getter methods for current PID values
    float getKp() { return kp; }
    float getKd() { return kd; }
    float getKi() { return ki; }
    float getUmax() { return umax; }
    
    // Compute the control signal
    void evalu(float value, float target, float deltaT, int &pwr, int &dir) {
      // Error
      float e = target - value;
      
      // Derivative
      float dedt = (e - eprev) / deltaT;
      
      // Integral
      eintegral = eintegral + e * deltaT;
      
      // Control signal
      float u = kp * e + kd * dedt + ki * eintegral;
      
      // Motor power
      pwr = (int)fabs(u);
      if (pwr > umax) {
        pwr = umax;
      }
      
      // Motor direction
      dir = 1;
      if (u < 0) {
        dir = -1;
      }
      
      // Store previous value of error
      eprev = e;
    }
    
    // Reset the controller's integral and previous error terms
    void reset() {
      eprev = 0.0;
      eintegral = 0.0;
    }
};

// Import Motor Driver Library
#include "CytronMotorDriver.h"

// Configure the motor drivers
CytronMD motor1(PWM_DIR, MOTOR_DRIVER_PWM1_PIN, MOTOR_DRIVER_DIR1_PIN);  // Left motor
CytronMD motor2(PWM_DIR, MOTOR_DRIVER_PWM2_PIN, MOTOR_DRIVER_DIR2_PIN);  // Right motor

// Motor direction adjustment if needed (set to -1 if motor runs in the opposite direction)
const int LEFT_MOTOR_DIRECTION = 1;   // Change to -1 if left motor runs backward
const int RIGHT_MOTOR_DIRECTION = 1;  // Change to -1 if right motor runs backward

// Encoder tracking variables
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
int lastLeftState = 0;
int lastRightState = 0;

// Timing variables
unsigned long lastUpdateTime = 0;
unsigned long lastPIDUpdateTime = 0;
unsigned long lastPlotTime = 0;
unsigned long startTime = 0;
const unsigned long PID_UPDATE_INTERVAL = 20; // PID update every 20ms for smoother control

// Distance tracking
float leftDistanceMM = 0.0;
float rightDistanceMM = 0.0;

// Velocity tracking
float leftVelocity = 0.0;  // mm/s
float rightVelocity = 0.0; // mm/s
unsigned long prevLeftCount = 0;
unsigned long prevRightCount = 0;
unsigned long velocityUpdateTime = 0;

// Target velocity variables
float leftTargetVelocity = 0.0;  // mm/s
float rightTargetVelocity = 0.0; // mm/s

// Direction tracking for velocity calculation
int lastLeftDir = 1;  // Initialize with forward direction
int lastRightDir = 1; // Initialize with forward direction

// PID controllers for left and right wheels
SimplePID leftPID;
SimplePID rightPID;

// Function to convert raw motor speed to velocity (mm/s)
float convertSpeedToVelocity(int speed) {
  const float MAX_VELOCITY = 200.0; // mm/s at maximum PWM
  return (speed * MAX_VELOCITY) / MAX_MOTOR_SPEED;
}

// Modified function to set motor speeds with direction
void setMotorSpeed(CytronMD &motor, int power, int direction) {
  // We'll directly use the Cytron's setSpeed function which accepts values from -255 to 255
  int speed = direction * power;
  
  // Ensure speed doesn't exceed motor controller's limits
  speed = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  
  // Use Cytron's setSpeed which accepts negative values for reverse direction
  motor.setSpeed(speed);
}

void setup() {
  // Initialize pins
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  
  // Initialize Serial Communication
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);
  inputString.reserve(200); // Reserve space for serial input
  
  // Initialize encoder states
  lastLeftState = digitalRead(LEFT_ENC_PIN);
  lastRightState = digitalRead(RIGHT_ENC_PIN);
  
  // Initialize motors (stopped)
  motor1.setSpeed(0);
  motor2.setSpeed(0);

  // Initialize PID controllers - MODIFY THESE VALUES TO TUNE
  // Parameters: Kp, Kd, Ki, max_output
  leftPID.setParams(1.0, 0.003, 0.006, MAX_MOTOR_SPEED);  // Increased derivative and integral gains
  rightPID.setParams(1.0, 0.003, 0.006, MAX_MOTOR_SPEED); // Increased derivative and integral gains
  
  // Initialize timing
  lastUpdateTime = millis();
  lastPIDUpdateTime = millis();
  lastPlotTime = millis();
  velocityUpdateTime = millis();
  startTime = millis();
  
  // Print welcome message and instructions
  Serial.println("ESP32 PID Motor Tuner");
  Serial.println("---------------------");
  Serial.println("Send commands in this format to tune PID:");
  Serial.println("p0.8 = Set Kp to 0.8");
  Serial.println("i0.01 = Set Ki to 0.01");
  Serial.println("d0.2 = Set Kd to 0.2");
  Serial.println("stats = Show current PID values");
  Serial.println("---------------------");
  
  // Print header for serial plotter
  Serial.println("leftTarget,leftVelocity,rightTarget,rightVelocity,leftPower,rightPower");
}

void loop() {
  // Check for serial commands
  checkSerialCommands();
  
  // Read encoder states and count transitions
  int currentLeftState = digitalRead(LEFT_ENC_PIN);
  int currentRightState = digitalRead(RIGHT_ENC_PIN);
  
  // Check for left encoder transition (count in both directions)
  if (currentLeftState != lastLeftState) {
    leftEncoderCount++;
    lastLeftState = currentLeftState;
  }
  
  // Check for right encoder transition (count in both directions)
  if (currentRightState != lastRightState) {
    rightEncoderCount++;
    lastRightState = currentRightState;
  }

  // Update sensor readings and velocities
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= 50) { // 50ms update interval
    // Calculate distances
    leftDistanceMM = calculateDistance(leftEncoderCount);
    rightDistanceMM = calculateDistance(rightEncoderCount);
    
    // Calculate velocities (counts per second, then convert to mm/s)
    float deltaTime = (currentTime - velocityUpdateTime) / 1000.0; // Convert to seconds
    unsigned long deltaLeftCount = leftEncoderCount - prevLeftCount;
    unsigned long deltaRightCount = rightEncoderCount - prevRightCount;
    
    // Calculate raw velocity (doesn't account for direction yet)
    float rawLeftVelocity = (deltaLeftCount / TRANSITIONS_PER_REV) * PI * WHEEL_DIAMETER_MM / deltaTime;
    float rawRightVelocity = (deltaRightCount / TRANSITIONS_PER_REV) * PI * WHEEL_DIAMETER_MM / deltaTime;
    
    // We need to track the last direction commands and use them to determine velocity direction
    // If last command was negative, we assume we're moving backward
    if (lastLeftDir < 0) {
      leftVelocity = -rawLeftVelocity;
    } else {
      leftVelocity = rawLeftVelocity;
    }
    
    if (lastRightDir < 0) {
      rightVelocity = -rawRightVelocity;
    } else {
      rightVelocity = rawRightVelocity;
    }
    
    // Update previous counts and time
    prevLeftCount = leftEncoderCount;
    prevRightCount = rightEncoderCount;
    velocityUpdateTime = currentTime;
    
    // Update timing
    lastUpdateTime = currentTime;
  }

  // Generate target velocity based on test mode
  float elapsedSec = (currentTime - startTime) / 1000.0;
  
  switch(TEST_MODE) {
    case 1: { // Sine wave - tests both positive and negative velocities
      leftTargetVelocity = 100.0 * sin(elapsedSec * 0.5);
      rightTargetVelocity = 100.0 * sin(elapsedSec * 0.5);
      break;
    }
    case 2: { // Step response - alternates between positive, zero, and negative
      int stepPhase = (int)(elapsedSec / 5) % 3;
      if (stepPhase == 0) {
        leftTargetVelocity = 80.0;
        rightTargetVelocity = 80.0;
      } else if (stepPhase == 1) {
        leftTargetVelocity = 0.0;
        rightTargetVelocity = 0.0;
      } else {
        leftTargetVelocity = -80.0;
        rightTargetVelocity = -80.0;
      }
      break;
    }
    case 3: { // Ramp - goes from positive to negative
      float cycleTime = fmod(elapsedSec, 10.0);
      if (cycleTime < 5.0) {
        // Ramp from -100 to +100
        leftTargetVelocity = (cycleTime - 2.5) * 40.0;
        rightTargetVelocity = (cycleTime - 2.5) * 40.0;
      } else {
        // Ramp from +100 to -100
        leftTargetVelocity = (7.5 - cycleTime) * 40.0;
        rightTargetVelocity = (7.5 - cycleTime) * 40.0;
      }
      break;
    }
  }

  // Variables to store PID outputs
  int leftPower = 0, leftDir = 0;
  int rightPower = 0, rightDir = 0;

  // Apply PID control to maintain target velocities
  if (currentTime - lastPIDUpdateTime >= PID_UPDATE_INTERVAL) {
    // Calculate time delta in seconds
    float deltaT = (currentTime - lastPIDUpdateTime) / 1000.0;
    
    // Compute PID outputs for left motor
    leftPID.evalu(leftVelocity, leftTargetVelocity, deltaT, leftPower, leftDir);
    
    // Compute PID outputs for right motor
    rightPID.evalu(rightVelocity, rightTargetVelocity, deltaT, rightPower, rightDir);
    
    // Apply the computed control signals to motors with direction adjustment
    setMotorSpeed(motor1, leftPower, leftDir * LEFT_MOTOR_DIRECTION);
    setMotorSpeed(motor2, rightPower, rightDir * RIGHT_MOTOR_DIRECTION);
    
    // Store the current direction for velocity calculation
    lastLeftDir = leftDir;
    lastRightDir = rightDir;
    
    lastPIDUpdateTime = currentTime;
  }

  // Plot data for visualization
  if (currentTime - lastPlotTime >= PLOT_INTERVAL) {
    // Format: leftTarget,leftVelocity,rightTarget,rightVelocity,leftPower,rightPower
    Serial.print(leftTargetVelocity);
    Serial.print(",");
    Serial.print(leftVelocity);
    Serial.print(",");
    Serial.print(rightTargetVelocity);
    Serial.print(",");
    Serial.print(rightVelocity);
    Serial.print(",");
    Serial.print(leftDir < 0 ? -leftPower : leftPower);  // Show power with direction
    Serial.print(",");
    Serial.println(rightDir < 0 ? -rightPower : rightPower); // Show power with direction
    
    lastPlotTime = currentTime;
  }
}

// Function to calculate distance from encoder counts
float calculateDistance(unsigned long counts) {
  float revolutions = counts / TRANSITIONS_PER_REV;
  return revolutions * PI * WHEEL_DIAMETER_MM;
}

// Function to handle serial commands
void checkSerialCommands() {
  // Process completed commands
  if (stringComplete) {
    // Convert to lowercase for easier parsing
    inputString.toLowerCase();
    inputString.trim();  // Remove any whitespace
    
    // Variables to store parsed parameter values
    float value = 0.0;
    
    // Check for p command (proportional gain)
    if (inputString.startsWith("p")) {
      value = inputString.substring(1).toFloat();
      if (value >= 0.0) {
        // Update both PID controllers with the new Kp value
        leftPID.setParams(value, leftPID.getKd(), leftPID.getKi(), MAX_MOTOR_SPEED);
        rightPID.setParams(value, rightPID.getKd(), rightPID.getKi(), MAX_MOTOR_SPEED);
        
        Serial.print("Kp set to: ");
        Serial.println(value);
      }
    }
    // Check for i command (integral gain)
    else if (inputString.startsWith("i")) {
      value = inputString.substring(1).toFloat();
      if (value >= 0.0) {
        // Update both PID controllers with the new Ki value
        leftPID.setParams(leftPID.getKp(), leftPID.getKd(), value, MAX_MOTOR_SPEED);
        rightPID.setParams(rightPID.getKp(), rightPID.getKd(), value, MAX_MOTOR_SPEED);
        
        Serial.print("Ki set to: ");
        Serial.println(value);
      }
    }
    // Check for d command (derivative gain)
    else if (inputString.startsWith("d")) {
      value = inputString.substring(1).toFloat();
      if (value >= 0.0) {
        // Update both PID controllers with the new Kd value
        leftPID.setParams(leftPID.getKp(), value, leftPID.getKi(), MAX_MOTOR_SPEED);
        rightPID.setParams(rightPID.getKp(), value, rightPID.getKi(), MAX_MOTOR_SPEED);
        
        Serial.print("Kd set to: ");
        Serial.println(value);
      }
    }
    else if (inputString.startsWith("m")) {
      value = inputString.substring(1).toInt();
      if (value >= 0) {
        // Update test mode
        TEST_MODE = value;
        Serial.print("Testmode set to: ");
        Serial.println(value);
      }
    }
    // Check for stats command
    else if (inputString.equals("stats")) {
      Serial.println("Current PID Parameters:");
      Serial.println("----------------------");
      Serial.print("Kp: ");
      Serial.println(leftPID.getKp());
      Serial.print("Kd: ");
      Serial.println(leftPID.getKd());
      Serial.print("Ki: ");
      Serial.println(leftPID.getKi());
      Serial.println("----------------------");
    }
    
    // Clear the input string for next command
    inputString = "";
    stringComplete = false;
  }
}

// Serial event occurs whenever new data comes in the hardware serial RX
void serialEvent() {
  while (Serial.available()) {
    // Get the new byte
    char inChar = (char)Serial.read();
    
    // Add it to the inputString
    if (inChar != '\n') {
      inputString += inChar;
    }
    // If the incoming character is a newline, set a flag so the main loop can
    // do something about it
    else {
      stringComplete = true;
    }
  }
}
