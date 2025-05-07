/*
  By Ranil Ganlath (Modified for ROS2 Communication)
  This script is for adapting the Serial Controller to be in a format for sending and receiving ROS2 serial messages.
  The Dark Passenger ROS2 Controller expects a command along with chunks of data like floats, text, etc. The data notation is "<CMD,value_1,value_2,...value_n>. Ex. <M,80,80> which sets the motor velocities for both wheels to 80 speed.

  ESP32 1  - Battery Input Pin (from 0 to 3.3V)
  ESP32 47  - Toggle Switch 1
  ESP32 48  - Toggle Switch 2 (Not working, ESP32 might be damaged since wiring is good)
  ESP32 35  - Relay FWD Headlight Control Output
  ESP32 36  - Relay AFT Brake Light Control Output
  ESP32 8  -  IMS I2C SDA
  ESP32 9  -  IMS I2C SCL
  ESP32 38  - GPS RX Input (routes to TX)
  ESP32 37  - GPS TX Output (routes to RX)
  ESP32 GND - Motor Driver GND
  ESP32 11  - Motor Driver PWM 1 Input
  ESP32 13  - Motor Driver DIR 1 Input
  ESP32 21  - Motor Driver PWM 2 Input
  ESP32 19  - Motor Driver DIR 2 Input
  
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

// Pin definitions
#define BATTERY_MONITOR_PIN 1
#define TOGGLE_SW1_PIN 47
#define TOGGLE_SW2_PIN 48 //not working
#define FWD_RELAY_CTRL_PIN 35
#define AFT_RELAY_CTRL_PIN 36
#define NEOPIXEL_PIN 48 //pin 42 is neopixel breakout pin.
#define IMS_SDA_PIN 8
#define IMS_SCL_PIN 9
#define GPS_RX_PIN 38
#define GPS_TX_PIN 37
#define MOTOR_DRIVER_PWM1_PIN 11
#define MOTOR_DRIVER_DIR1_PIN 13
#define MOTOR_DRIVER_PWM2_PIN 21
#define MOTOR_DRIVER_DIR2_PIN 19
#define LEFT_ENC_PIN 41
#define RIGHT_ENC_PIN 6
#define LEFT_WHEEL_SW_PIN 40
#define RIGHT_WHEEL_SW_PIN 5

// Motor speed constants
#define MAX_MOTOR_SPEED 100  // Maximum PWM value
#define MOTOR_SPEED 100      // Standard movement speed (25% of max)

// Encoder parameters
#define TRANSITIONS_PER_REV 508.8  // 8 transitions × 63.6:1 gear ratio
#define WHEEL_DIAMETER_MM 70.0     // Roomba wheel diameter in mm (adjust if different)
#define PI 3.14159265359

// ROS2 Communication Parameters
#define SERIAL_BAUD_RATE 115200
#define MSG_START_MARKER '<'
#define MSG_END_MARKER '>'
#define MSG_SEPARATOR ','
#define PUBLISH_INTERVAL 200     // Publish sensor data every 200ms

// ROS2 Command IDs
#define CMD_MOVE 'M'          // Format: <M,speed_left,speed_right>
#define CMD_LIGHTS 'L'        // Format: <L,headlight,brakelight> (0=off, 1=on)
#define CMD_RESET_ENC 'R'     // Format: <R> - Reset encoders
#define CMD_HEARTBEAT 'H'     // Format: <H> - Triggers MSG_Start, Responds with <K,1>


// ROS2 Sensor Message Types
#define MSG_START 'K'         // Format: <K,1>
#define MSG_ODOM 'O'          // Format: <O,left_dist,right_dist,left_count,right_count>
#define MSG_BATTERY 'B'       // Format: <B,voltage,percentage>
#define MSG_IMU 'I'           // Format: <I,aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,heading>
#define MSG_GPS 'G'           // Format: <G,lat,lng,satellites,valid>
#define MSG_STATUS 'S'        // Format: <S,sw1,sw2,wheel1_sw,wheel2_sw,headlight,brakelight>

//Configure GPS
#define GPS_BAUD 9600
HardwareSerial gpsSerial(2);


// class for computing motor control signal
class SimplePID {
  private:
    float kp, kd, ki, umax; //Params for pid gains and max motor speed
    float eprev, eintegral; //Storage for derivative and integral from prior loop
  public:
    //Constructor
    SimplePID(): kp(1.0), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    //Function to set params
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }
    //compute the control signal
    void evalu(float value, float target, float deltaT, int &pwr, int &dir) {
      //this is the equation we are trying to solve: u(t) = Kp*e(t) + Ki* integral(e(t)*dt) + Kd* de/dt
      //Kp is proportial gain, Ki is integral gain, Kd is derivative gain
      //e(t) is the error value, de is the change in error value, dt is change in time
      //We solve for u(t) which is the PID control variable which converts to motor speed and direction
      //error
      float e = target - value;
      //derivative
      float dedt = (e - eprev) / (deltaT); //change in error over change in time
      //integral
      eintegral = eintegral + e * deltaT;
      //control signal
      float u = kp * e + kd * dedt + ki * eintegral;
      //motor power
      pwr = (int) fabs(u); //take magnitude and store in pointer
      if ( pwr > umax ) { //boundary check
        pwr = umax;
      }
      //motor direction
      dir = 1;
      if (u < 0) {
        dir = -1;
      }
      //store previous value of error
      eprev = e;
    }
    
    // Reset the controller's integral and previous error terms
    void reset() {
      eprev = 0.0;
      eintegral = 0.0;
    }
};


//Import Libraries
#include "CytronMotorDriver.h"
#include <TinyGPSPlus.h>
#include <MPU9250_asukiaaa.h>

//Configure GPS
TinyGPSPlus gps;
const int timeZoneOffset = -6;

//Configure IMS MPU9250
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

// Configure the motor driver.
CytronMD motor1(PWM_DIR, MOTOR_DRIVER_PWM1_PIN, MOTOR_DRIVER_DIR1_PIN);
CytronMD motor2(PWM_DIR, MOTOR_DRIVER_PWM2_PIN, MOTOR_DRIVER_DIR2_PIN);

// Motor direction adjustment if needed (set to -1 if motor runs in the opposite direction)
const int LEFT_MOTOR_DIRECTION = -1;   // Change to -1 if left motor runs backward
const int RIGHT_MOTOR_DIRECTION = -1;  // Change to -1 if right motor runs backward


// Encoder tracking variables
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
int lastLeftState = 0;
int lastRightState = 0;

// Timing variables
unsigned long lastUpdateTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastPIDUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // Update every 100ms
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

// ROS2 Communication variables
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;
boolean receivingData = false;
byte dataIndex = 0;

//Battery Level Monitoring
int batteryVoltage = 3000; // actual battery voltage in millivolts
int batteryPercentage = 100;

//Configure Relays
bool FWD_RELAY_ON = false;
bool AFT_RELAY_ON = false;

//Configure Toggle Switches
int toggleSwitch1_state = 0;        // value read from SW1
int toggleSwitch2_state = 0;        // value read from SW2


// Function to convert raw motor speed to velocity (mm/s)
float convertSpeedToVelocity(int speed) {
  // Simple linear conversion from motor PWM to velocity
  // This is an approximation and should be calibrated for your specific robot
  // Maximum speed of 255 PWM might correspond to approximately 200 mm/s
  const float MAX_VELOCITY = 200.0; // mm/s at maximum PWM
  return (speed * MAX_VELOCITY) / MAX_MOTOR_SPEED;
}

// Function to set motor speeds with direction
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
  pinMode(LEFT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(FWD_RELAY_CTRL_PIN, OUTPUT);
  pinMode(AFT_RELAY_CTRL_PIN, OUTPUT);
  pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP); 

  // Initialize relays (turn off)
  digitalWrite(FWD_RELAY_CTRL_PIN, LOW);
  digitalWrite(AFT_RELAY_CTRL_PIN, LOW);
  
  // Battery Monitoring
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  analogReadResolution(12);
  
  // Initialize Serial Communication
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);

  // IMS I2C initialization
  Wire.begin(IMS_SDA_PIN, IMS_SCL_PIN);
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // GPS initialization
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Initialize encoder states
  lastLeftState = digitalRead(LEFT_ENC_PIN);
  lastRightState = digitalRead(RIGHT_ENC_PIN);
  
  // Initialize motors (stopped)
  motor1.setSpeed(0);
  motor2.setSpeed(0);

  // Initialize PID controllers
  // Parameters: Kp, Kd, Ki, max_output
  leftPID.setParams(1.0, 0.003, 0.006, MAX_MOTOR_SPEED);
  rightPID.setParams(1.0, 0.003, 0.006, MAX_MOTOR_SPEED);
  //Note incorporate speed-dependent tuning later

   
  // Send robot initialization complete message  
  Serial.println("<K,1>");

  // Initialize timing
  lastUpdateTime = millis();
  lastPIDUpdateTime = millis();
  velocityUpdateTime = millis();
}

void loop() {
  // Process GPS data continuously
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Check for incoming ROS2 commands
  receiveROS2Message();

  // Process new ROS2 commands
  if (newData) {
    parseAndExecuteCommand();
    newData = false;
  }

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

  // Update sensor readings periodically
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
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
    
    // Read other sensors
    readBatteryLevel();
    readToggleSwitches();
    updateIMSData();
    
    // Update timing
    lastUpdateTime = currentTime;
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


  // Publish sensor data to ROS2 at the specified interval
  if (currentTime - lastPublishTime >= PUBLISH_INTERVAL) {
    publishOdometryData();
    publishBatteryData();
    publishIMUData();
    publishGPSData();
    publishStatusData();
    
    lastPublishTime = currentTime;
  }
}

// Function to receive ROS2 messages (format: <CMD,param1,param2,...>)
void receiveROS2Message() {
  if (Serial.available() > 0) {
    char rc = Serial.read();
    
    // Start of message
    if (rc == MSG_START_MARKER) {
      receivingData = true;
      dataIndex = 0;
      memset(receivedChars, 0, numChars);
    }
    // End of message
    else if (rc == MSG_END_MARKER) {
      receivingData = false;
      newData = true;
    }
    // Message content
    else if (receivingData) {
      if (dataIndex < numChars - 1) {
        receivedChars[dataIndex] = rc;
        dataIndex++;
      }
    }
  }
}

// Function to parse and execute received commands
void parseAndExecuteCommand() {
  // First character is the command type
  char cmdType = receivedChars[0];
  
  // Parse parameters (splitting by commas)
  int paramIndex = 0;
  char* params[10]; // Can handle up to 10 parameters
  
  // Skip the command type
  char* token = strtok(receivedChars + 1, ",");
  
  // Parse parameters
  while (token != NULL && paramIndex < 10) {
    params[paramIndex++] = token;
    token = strtok(NULL, ",");
  }
  
  // Execute command based on type
  switch (cmdType) {
    case CMD_MOVE:
      if (paramIndex >= 2) {
        int speedLeft = atoi(params[0]);
        int speedRight = atoi(params[1]);
        // Constrain speeds between -MOTOR_SPEED and MOTOR_SPEED
        speedLeft = constrain(speedLeft, -MOTOR_SPEED, MOTOR_SPEED);
        speedRight = constrain(speedRight, -MOTOR_SPEED, MOTOR_SPEED);
        
        // Convert speed commands to target velocities
        leftTargetVelocity = convertSpeedToVelocity(speedLeft);
        rightTargetVelocity = convertSpeedToVelocity(speedRight);
        
        // If the target velocity is zero, reset the PID to prevent integral windup
        if (leftTargetVelocity == 0) leftPID.reset();
        if (rightTargetVelocity == 0) rightPID.reset();
      }
      break;
      
    case CMD_LIGHTS:
      if (paramIndex >= 2) {
        int headlight = atoi(params[0]);
        int brakelight = atoi(params[1]);
        // Set lights
        if (headlight == 1) turnOnHeadlight();
        else turnOffHeadlight();
        
        if (brakelight == 1) turnOnBrakelight();
        else turnOffBrakelight();
      }
      break;
      
    case CMD_RESET_ENC:
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      leftDistanceMM = 0;
      rightDistanceMM = 0;
      prevLeftCount = 0;
      prevRightCount = 0;
      // Reset PID controllers
      leftPID.reset();
      rightPID.reset();
      break;

    case CMD_HEARTBEAT:
      Serial.println("<K,1>");
      break;
  }
}

// Function to publish odometry data
void publishOdometryData() {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "<%c,%ld,%ld,%.2f,%.2f>", 
           MSG_ODOM, 
           leftEncoderCount, 
           rightEncoderCount, 
           leftDistanceMM, 
           rightDistanceMM);
  Serial.println(buffer);
}

// Function to publish battery data
void publishBatteryData() {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "<%c,%d,%d>", 
           MSG_BATTERY, 
           batteryVoltage, 
           batteryPercentage);
  Serial.println(buffer);
}

// Function to publish IMU data
void publishIMUData() {
  char buffer[96];
  snprintf(buffer, sizeof(buffer), "<%c,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f>", 
           MSG_IMU, 
           aX, aY, aZ, 
           gX, gY, gZ, 
           mX, mY, mZ, 
           mDirection);
  Serial.println(buffer);
}

// Function to publish GPS data
void publishGPSData() {
  char buffer[64];
  int valid = gps.location.isValid() ? 1 : 0;
  snprintf(buffer, sizeof(buffer), "<%c,%.6f,%.6f,%d,%d>", 
           MSG_GPS, 
           gps.location.isValid() ? gps.location.lat() : 0.0, 
           gps.location.isValid() ? gps.location.lng() : 0.0, 
           gps.satellites.isValid() ? gps.satellites.value() : 0,
           valid);
  Serial.println(buffer);
}

// Function to publish status data
void publishStatusData() {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "<%c,%d,%d,%d,%d,%d,%d>", 
           MSG_STATUS, 
           toggleSwitch1_state, 
           toggleSwitch2_state,
           !digitalRead(LEFT_WHEEL_SW_PIN),
           !digitalRead(RIGHT_WHEEL_SW_PIN),
           FWD_RELAY_ON ? 1 : 0,
           AFT_RELAY_ON ? 1 : 0);
  Serial.println(buffer);
}

// Function to update IMU sensor data
void updateIMSData() {
  // Update accelerometer
  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
  }
  
  // Update gyroscope
  if (mySensor.gyroUpdate() == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
  }
  
  // Update magnetometer
  if (mySensor.magUpdate() != 0) {
    mySensor.beginMag();
    mySensor.magUpdate();
  }
  
  // Read magnetometer values
  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
  }
}

// Function to calculate distance from encoder counts
float calculateDistance(unsigned long counts) {
  float revolutions = counts / TRANSITIONS_PER_REV;
  return revolutions * PI * WHEEL_DIAMETER_MM;
}

// Function to check the battery level
void readBatteryLevel() {
  batteryVoltage = analogReadMilliVolts(BATTERY_MONITOR_PIN);
  batteryPercentage = map(batteryVoltage, 2570, 3050, 0, 100);
  if(batteryPercentage > 100) batteryPercentage = 100;
  if(batteryPercentage < 0) batteryPercentage = 0; 
}

// Function to read toggle switches
void readToggleSwitches() {
  toggleSwitch1_state = digitalRead(TOGGLE_SW1_PIN);
  toggleSwitch2_state = digitalRead(TOGGLE_SW2_PIN);
}

// Function to turn on headlight relay
void turnOnHeadlight() {
  digitalWrite(FWD_RELAY_CTRL_PIN, HIGH);
  FWD_RELAY_ON = true;
}

// Function to turn off headlight relay
void turnOffHeadlight() {
  digitalWrite(FWD_RELAY_CTRL_PIN, LOW);
  FWD_RELAY_ON = false;
}

// Function to toggle headlight relay
void toggleHeadlight() {
  if(FWD_RELAY_ON) {
    digitalWrite(FWD_RELAY_CTRL_PIN, LOW);
    FWD_RELAY_ON = false;
  } else {
    digitalWrite(FWD_RELAY_CTRL_PIN, HIGH);
    FWD_RELAY_ON = true;
  }
}

// Function to turn on brakelight relay
void turnOnBrakelight() {
  digitalWrite(AFT_RELAY_CTRL_PIN, HIGH);
  AFT_RELAY_ON = true;
}

// Function to turn off brakelight relay
void turnOffBrakelight() {
  digitalWrite(AFT_RELAY_CTRL_PIN, LOW);
  AFT_RELAY_ON = false;
}

// Function to toggle brakelight relay
void toggleBrakelight() {
  if(AFT_RELAY_ON) {
    digitalWrite(AFT_RELAY_CTRL_PIN, LOW);
    AFT_RELAY_ON = false;
  } else {
    digitalWrite(AFT_RELAY_CTRL_PIN, HIGH);
    AFT_RELAY_ON = true;
  }
}
