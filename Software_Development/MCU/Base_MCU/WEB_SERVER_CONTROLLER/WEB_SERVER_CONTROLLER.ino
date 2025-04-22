/*
  Robot Web Controller - By adapting code from Ranil Ganlath
  This script allows controlling a robot via a web interface instead of serial commands
  
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

// WiFi settings
#include "credentials.h"  // Include WiFi credentials
#include "webpage.h"      // HTML webpage template
#include <WiFi.h>
WiFiServer server(80);

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

//Import Libraries
#include "CytronMotorDriver.h"
#include <TinyGPSPlus.h>
#include <MPU9250_asukiaaa.h>

//Configure GPS
#define GPS_BAUD 9600
HardwareSerial gpsSerial(2);
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

// Battery Level Monitoring
int batteryVoltage = 3000; // actual battery voltage in millivolts
int batteryPercentage = 100;

// Configure Relays
bool FWD_RELAY_ON = false;
bool AFT_RELAY_ON = false;

// Configure Toggle Switches
int toggleSwitch1_state = 0;        // value read from SW1
int toggleSwitch2_state = 0;        // value read from SW2

// Data collection for web display
String gpsData = "No GPS data available";
String imsData = "No IMS data available";
String encoderData = "No encoder data available";
String batteryData = "Battery: 100%";



// The setup routine runs once when you press reset.
void setup() {
  // Initialize all pins
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  pinMode(LEFT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(FWD_RELAY_CTRL_PIN, OUTPUT);
  pinMode(AFT_RELAY_CTRL_PIN, OUTPUT);
  pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP); 

  // Initialize relays to OFF
  digitalWrite(FWD_RELAY_CTRL_PIN, LOW);
  digitalWrite(AFT_RELAY_CTRL_PIN, LOW);
  
  // Battery Monitoring
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  analogReadResolution(12);
  
  Serial.begin(115200);
  while(!Serial);

  // Set up WiFi connection
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  server.begin();

  // IMS I2C setup
  Wire.begin(IMS_SDA_PIN, IMS_SCL_PIN);
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // GPS setup
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Initialize last encoder states
  lastLeftState = digitalRead(LEFT_ENC_PIN);
  lastRightState = digitalRead(RIGHT_ENC_PIN);
  
  Serial.println("Robot Web Controller Started");
  Serial.println("Web interface available at: http://" + WiFi.localIP().toString());
  
  // Initially stopped
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  
  // Initial readings
  readBatteryLevel();
}

// The loop routine runs over and over again
void loop() {
  // Update sensor readings
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());
    
  // Check encoder states and count transitions
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

  // Update and calculate distances periodically
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

    // Miscellaneous Periodic Monitoring
    readBatteryLevel();
    
    // Update battery data for web display
    batteryData = "Battery: " + String(batteryPercentage) + "% (" + String(batteryVoltage) + "mV)";
  }

  // Handle web clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);  // Print to serial monitor
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // HTTP header has ended, send a response
            sendWebPage(client);
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
        
        // Process commands from web interface
        if (currentLine.endsWith("GET /F")) processCommand('F');
        else if (currentLine.endsWith("GET /B")) processCommand('B');
        else if (currentLine.endsWith("GET /L")) processCommand('L');
        else if (currentLine.endsWith("GET /R")) processCommand('R');
        else if (currentLine.endsWith("GET /S")) processCommand('S');
        else if (currentLine.endsWith("GET /H")) processCommand('H');
        else if (currentLine.endsWith("GET /J")) processCommand('J');
        else if (currentLine.endsWith("GET /C")) processCommand('C');
        else if (currentLine.endsWith("GET /K")) processCommand('K');
        else if (currentLine.endsWith("GET /G")) processCommand('G');
        else if (currentLine.endsWith("GET /E")) processCommand('E');
        else if (currentLine.endsWith("GET /T")) processCommand('T');
        else if (currentLine.endsWith("GET /I")) processCommand('I');
      }
    }
    
    client.stop();
    Serial.println("Client disconnected");
  }
}

// Send the web page with current data to the client
void sendWebPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  
  // Replace placeholders with current data
  String webpage = FPSTR(webpageTemplate);
  webpage.replace("%BATTERY%", batteryData);
  webpage.replace("%ENCODER%", encoderData);
  webpage.replace("%GPS%", gpsData);
  webpage.replace("%IMS%", imsData);
  
  client.print(webpage);
}

// Process received commands (same as the original serial commands)
void processCommand(char cmd) {
  command = cmd;
  
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
      encoderData = "Encoder counts cleared";
      break;
      
    case 'K':  // Report Battery Percentage
      Serial.println("Reporting Battery Voltage");
      Serial.printf("Battery voltage = %d mV\n", batteryVoltage);
      Serial.printf("Battery Percentage = %d Percent\n", batteryPercentage);
      break;
      
    case 'G':  // Report GPS Info
      Serial.println("Reporting GPS Info");
      readGPS();
      break;
      
    case 'E':  // Report Encoder Distance
      Serial.println("Reporting Distance Travelled");
      captureEncoderData();
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

///////////////////Function Definitions//////////////////
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
  if(batteryPercentage < 20) {
    Serial.println("Warning Battery Low!");
  }
}

// Function to check MPU9250 IMS
void readIMS() {
  String imsOutput = "<h3>IMS (Inertial Measurement System) Data:</h3>";
  uint8_t sensorId;
  int result;
  
  result = mySensor.readId(&sensorId);
  if (result == 0) {
    imsOutput += "<p>Sensor ID: " + String(sensorId) + "</p>";
  } else {
    imsOutput += "<p>Cannot read sensor ID: " + String(result) + "</p>";
  }
  
  result = mySensor.accelUpdate();
  if (result == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    imsOutput += "<p>Acceleration:<br>";
    imsOutput += "X: " + String(aX) + "<br>";
    imsOutput += "Y: " + String(aY) + "<br>";
    imsOutput += "Z: " + String(aZ) + "<br>";
    imsOutput += "Magnitude: " + String(aSqrt) + "</p>";
  } else {
    imsOutput += "<p>Cannot read acceleration values: " + String(result) + "</p>";
  }
  
  result = mySensor.gyroUpdate();
  if (result == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    imsOutput += "<p>Gyroscope:<br>";
    imsOutput += "X: " + String(gX) + "<br>";
    imsOutput += "Y: " + String(gY) + "<br>";
    imsOutput += "Z: " + String(gZ) + "</p>";
  } else {
    imsOutput += "<p>Cannot read gyro values: " + String(result) + "</p>";
  }
  
  result = mySensor.magUpdate();
  if (result != 0) {
    imsOutput += "<p>Cannot read mag, reinitiating magnetometer...</p>";
    mySensor.beginMag();
    result = mySensor.magUpdate();
  }
  
  if (result == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    imsOutput += "<p>Magnetometer:<br>";
    imsOutput += "X: " + String(mX) + "<br>";
    imsOutput += "Y: " + String(mY) + "<br>";
    imsOutput += "Z: " + String(mZ) + "<br>";
    imsOutput += "Direction: " + String(mDirection) + "°</p>";
  } else {
    imsOutput += "<p>Cannot read magnetometer values: " + String(result) + "</p>";
  }
  
  imsData = imsOutput;
  Serial.println("IMS data updated");
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

void readToggleSwitches() {
  toggleSwitch1_state = digitalRead(TOGGLE_SW1_PIN);
  toggleSwitch2_state = digitalRead(TOGGLE_SW2_PIN);
}

void captureEncoderData() {
  String output = "<h3>Distance Data:</h3>";
  
  output += "<p>Encoder Counts:<br>";
  output += "Left: " + String(leftEncoderCount) + "<br>";
  output += "Right: " + String(rightEncoderCount) + "</p>";
  
  output += "<p>Distance (Metric):<br>";
  output += "Left: " + String(leftDistanceMM, 1) + " mm (" + String(leftDistanceMM/1000, 2) + " m)<br>";
  output += "Right: " + String(rightDistanceMM, 1) + " mm (" + String(rightDistanceMM/1000, 2) + " m)<br>";
  output += "Average: " + String(avgDistanceMM, 1) + " mm (" + String(avgDistanceMM/1000, 2) + " m)</p>";
  
  output += "<p>Distance (Imperial):<br>";
  output += "Left: " + String(leftDistanceInches, 1) + " in (" + String(leftDistanceInches/12, 2) + " ft)<br>";
  output += "Right: " + String(rightDistanceInches, 1) + " in (" + String(rightDistanceInches/12, 2) + " ft)<br>";
  output += "Average: " + String(avgDistanceInches, 1) + " in (" + String(avgDistanceInches/12, 2) + " ft)</p>";
  
  output += "<p>Wheel Contact Switches:<br>";
  output += "Left: " + String(digitalRead(LEFT_WHEEL_SW_PIN)) + "<br>";
  output += "Right: " + String(digitalRead(RIGHT_WHEEL_SW_PIN)) + "</p>";
  
  encoderData = output;
  Serial.println("Encoder data updated");
}

// Function to read and display GPS data
void readGPS() {
  String output = "<h3>GPS Data:</h3>";
  
  if (gps.satellites.isValid()) {
    output += "<p>Satellites: " + String(gps.satellites.value()) + "</p>";
  } else {
    output += "<p>Satellites: No data</p>";
  }
  
  if (gps.location.isValid()) {
    output += "<p>Location:<br>";
    output += "Latitude: " + String(gps.location.lat(), 6) + "<br>";
    output += "Longitude: " + String(gps.location.lng(), 6) + "</p>";
  } else {
    output += "<p>Location: No data</p>";
  }
  
  if (gps.date.isValid()) {
    output += "<p>Date: ";
    output += String(gps.date.month()) + "/";
    output += String(gps.date.day()) + "/";
    output += String(gps.date.year()) + "</p>";
  } else {
    output += "<p>Date: No data</p>";
  }
  
  if (gps.time.isValid()) {
    int hour = gps.time.hour() + timeZoneOffset;
    // Handle day boundary crossing
    if (hour < 0) {
      hour += 24;
    } else if (hour >= 24) {
      hour -= 24;
    }
    
    output += "<p>Time (MNT): ";
    if (hour < 10) output += "0";
    output += String(hour) + ":";
    
    if (gps.time.minute() < 10) output += "0";
    output += String(gps.time.minute()) + ":";
    
    if (gps.time.second() < 10) output += "0";
    output += String(gps.time.second()) + "</p>";
  } else {
    output += "<p>Time: No data</p>";
  }
  
  if (gps.charsProcessed() < 10) {
    output += "<p>WARNING: No GPS data received - check wiring</p>";
  }
  
  gpsData = output;
  Serial.println("GPS data updated");
}
