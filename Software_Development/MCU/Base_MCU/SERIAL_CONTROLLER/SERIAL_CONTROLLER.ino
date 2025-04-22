/*
  By Ranil Ganlath
  This script is for merging the testing scripts into one serially controlled script. Once verified working, this script will be used to integrate with ROS.
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
#define MOTOR_SPEED 80      // Standard movement speed (25% of max)
#define TURNING_SPEED 80    // Speed for turns

// Encoder parameters
#define TRANSITIONS_PER_REV 508.8  // 8 transitions × 63.6:1 gear ratio
#define WHEEL_DIAMETER_MM 70.0     // Roomba wheel diameter in mm (adjust if different)
#define PI 3.14159265359
#define MM_TO_INCHES 0.0393701     // Conversion factor from mm to inches

//Configure GPS
#define GPS_BAUD 9600
HardwareSerial gpsSerial(2);


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

// Encoder tracking variables
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
int lastLeftState = 0;
int lastRightState = 0;
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // Update every 100ms

char command = 'S';         // Default Command to Stop

// Distance tracking - moved to global scope
float leftDistanceMM = 0.0;
float rightDistanceMM = 0.0;
float avgDistanceMM = 0.0;
float leftDistanceInches = 0.0;
float rightDistanceInches = 0.0;
float avgDistanceInches = 0.0;




//Battery Level Monitoring
int batteryVoltage = 3000; // actual battery voltage in volts
int batteryPercentage = 100;


//Configure Relays
bool FWD_RELAY_ON = false;
bool AFT_RELAY_ON = false;

//Configure Toggle Switches
int toggleSwitch1_state = 0;        // value read from SW1
int toggleSwitch2_state = 0;        // value read from SW2

// The setup routine runs once when you press reset.
void setup() {
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  pinMode(LEFT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(FWD_RELAY_CTRL_PIN,OUTPUT);
  pinMode(AFT_RELAY_CTRL_PIN,OUTPUT);
  pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP); 

  //Relay
  digitalWrite(FWD_RELAY_CTRL_PIN,HIGH);
  digitalWrite(AFT_RELAY_CTRL_PIN,HIGH);
  delay(5000);
  digitalWrite(FWD_RELAY_CTRL_PIN,LOW);
  digitalWrite(AFT_RELAY_CTRL_PIN,LOW);
  
  //Battery Monitoring
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  analogReadResolution(12);
  
  Serial.begin(115200);
  while(!Serial);

  //IMS I2C
  Wire.begin(IMS_SDA_PIN, IMS_SCL_PIN);
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  //GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Initialize last encoder states
  lastLeftState = digitalRead(LEFT_ENC_PIN);
  lastRightState = digitalRead(RIGHT_ENC_PIN);
  
  Serial.println("Roomba Serial Controller Started");
  Serial.println("Commands: F (forward), B (backward), R (right), L (left), S (stop), C (clear counters)");
  Serial.println("Encoder resolution: 508.8 transitions per wheel revolution");
  
  // Initially stopped
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

// The loop routine runs over and over again forever.
void loop() {
  // Process GPS data continuously
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

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
      case 'K':  // Report Battery Percentage
        Serial.println("Reporting Battery Voltage");
        Serial.printf("Battery voltage = %d V\n", batteryVoltage);
        Serial.printf("Battery Percentage = %d Percent\n", batteryPercentage);
        break;
      case 'G':  // Report GPS Info
        Serial.println("Reporting GPS Info");
        readGPS();
        break;
      case 'E':  // Report Encoder Distance
        Serial.println("Reporting Distance Travelled");
        printEncoders();
        break;
      case 'T':  // Toggle Switch
        Serial.println("Toggle Switch Read");
        readToggleSwitches();
        Serial.println("Toggle Switch 1 Value = " + String(toggleSwitch1_state) + "  Toggle Switch 2 Value =  "+ String(toggleSwitch2_state));
        break;
      case 'H':  // Headlight
        Serial.println("Toggling Headlight");
        toggleHeadlight();
        break;
      case 'J':  // Brake Light
        Serial.println("Toggling Brake Light");
        toggleBrakelight();
        break;
      case 'I':  // IMS Report
        Serial.println("Reporting IMS Readout");
        readIMS();
        break;
      default:
        // Do nothing for unrecognized commands
        break;
    }
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

  // Update and calculate distances periodically but don't print
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    // Calculate distances
    leftDistanceMM = calculateDistance(leftEncoderCount);
    rightDistanceMM = calculateDistance(rightEncoderCount);
    
    // Calculate average distance
    avgDistanceMM = (leftDistanceMM + rightDistanceMM) / 2;
    
    // Convert to inches
    leftDistanceInches = leftDistanceMM * MM_TO_INCHES;
    rightDistanceInches = rightDistanceMM * MM_TO_INCHES;
    avgDistanceInches = avgDistanceMM * MM_TO_INCHES;
    
    // Update the timing
    lastUpdateTime = currentTime;

    //Misc Periodic Monitoring
    readBatteryLevel();
  }
}


///////////////////Function Definitions//////////////////
// Function to calculate distance from encoder counts
float calculateDistance(unsigned long counts) {
  float revolutions = counts / TRANSITIONS_PER_REV;
  return revolutions * PI * WHEEL_DIAMETER_MM;
}

// Function to check the 20V battery level
void readBatteryLevel(){
  batteryVoltage = analogReadMilliVolts(BATTERY_MONITOR_PIN);
  batteryPercentage = map(batteryVoltage, 2570, 3050, 0, 100);
  if(batteryPercentage>100){batteryPercentage=100;}
  if(batteryPercentage<0){batteryPercentage=0;} 
  if(batteryPercentage<20){
    Serial.println("Warning Battery Low!");
  }
  return;
}

//Function to check MPU9250 IMS
void readIMS(){
  uint8_t sensorId;
  int result;
  result = mySensor.readId(&sensorId);
  if (result == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId " + String(result));
  }
  result = mySensor.accelUpdate();
  if (result == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.println("accelX: " + String(aX));
    Serial.println("accelY: " + String(aY));
    Serial.println("accelZ: " + String(aZ));
    Serial.println("accelSqrt: " + String(aSqrt));
  } else {
    Serial.println("Cannod read accel values " + String(result));
  }
  result = mySensor.gyroUpdate();
  if (result == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.println("gyroX: " + String(gX));
    Serial.println("gyroY: " + String(gY));
    Serial.println("gyroZ: " + String(gZ));
  } else {
    Serial.println("Cannot read gyro values " + String(result));
  }
  result = mySensor.magUpdate();
  if (result != 0) {
    Serial.println("cannot read mag so call begin again");
    mySensor.beginMag();
    result = mySensor.magUpdate();
  }
  if (result == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.println("magX: " + String(mX));
    Serial.println("maxY: " + String(mY));
    Serial.println("magZ: " + String(mZ));
    Serial.println("horizontal direction: " + String(mDirection));
  } else {
    Serial.println("Cannot read mag values " + String(result));
  }
  Serial.println("at " + String(millis()) + "ms");
  Serial.println(""); // Add an empty line
}



// Function to turn on headlight relay
void turnOnHeadlight(){
  digitalWrite(FWD_RELAY_CTRL_PIN,HIGH);
  FWD_RELAY_ON=true;
  return;
}

// Function to turn off headlight relay
void turnOffHeadlight(){
  digitalWrite(FWD_RELAY_CTRL_PIN,LOW);
  FWD_RELAY_ON=false;
  return;
}

// Function to toggle headlight relay
void toggleHeadlight(){
  if(FWD_RELAY_ON){
    digitalWrite(FWD_RELAY_CTRL_PIN,LOW);
    FWD_RELAY_ON=false;
  }
  else{
    digitalWrite(FWD_RELAY_CTRL_PIN,HIGH);
    FWD_RELAY_ON=true;
  }
  return;
}

// Function to turn on brakelight relay
void turnOnBrakelight(){
  digitalWrite(AFT_RELAY_CTRL_PIN,HIGH);
  AFT_RELAY_ON=true;
  return;
}

// Function to turn off brakelight relay
void turnOffBrakelight(){
  digitalWrite(AFT_RELAY_CTRL_PIN,LOW);
  AFT_RELAY_ON=false;
  return;
}

// Function to toggle brakelight relay
void toggleBrakelight(){
  if(AFT_RELAY_ON){
    digitalWrite(AFT_RELAY_CTRL_PIN,LOW);
    AFT_RELAY_ON=false;
  }
  else{
    digitalWrite(AFT_RELAY_CTRL_PIN,HIGH);
    AFT_RELAY_ON=true;
  }
  return;
}

void readToggleSwitches(){
  toggleSwitch1_state = digitalRead(TOGGLE_SW1_PIN);
  toggleSwitch2_state = digitalRead(TOGGLE_SW2_PIN);
  return;
}

void printEncoders(){
    // Print sensor and encoder data
  Serial.println("-------- Sensor Data --------");
  Serial.println("Left Wheel Switch: " + String(digitalRead(LEFT_WHEEL_SW_PIN)) + 
                ", Right Wheel Switch: " + String(digitalRead(RIGHT_WHEEL_SW_PIN)));

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
}


// Function to read and print GPS data
void readGPS() {
  // Print the essential GPS data
  if (gps.satellites.isValid()) {
    Serial.print(F("Satellites: "));
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("Satellites: *"));
  }
  
  Serial.print(F(" | "));
  
  if (gps.location.isValid()) {
    Serial.print(F("Lat: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" | Lng: "));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("Lat: * | Lng: *"));
  }
  
  Serial.print(F(" | "));
  
  // Date
  if (gps.date.isValid()) {
    Serial.print(F("Date: "));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("Date: *"));
  }
  
  Serial.print(F(" | "));
  
  // Time (adjusted for MNT)
  if (gps.time.isValid()) {
    Serial.print(F("Time (MNT): "));
    
    // Adjust hour for MNT
    int hour = gps.time.hour() + timeZoneOffset;
    
    // Handle day boundary crossing
    if (hour < 0) {
      hour += 24;
    } else if (hour >= 24) {
      hour -= 24;
    }
    
    if (hour < 10) Serial.print(F("0"));
    Serial.print(hour);
    Serial.print(F(":"));
    
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
  } else {
    Serial.print(F("Time (MNT): *"));
  }
  
  Serial.println();
  
  // Check if no GPS data is being received
  if (gps.charsProcessed() < 10)
    Serial.println(F("WARNING: No GPS data received - check wiring"));
}
