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
CytronMD motor2(PWM_DIR, 21, 19); // PWM 2 = Pin 21, DIR 2 = Pin 19.
#define LEFT_ENC_PIN 41
#define RIGHT_ENC_PIN 6
#define LEFT_WHEEL_SW_PIN 40
#define RIGHT_WHEEL_SW_PIN 5
// The setup routine runs once when you press reset.
void setup() {
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  pinMode(LEFT_WHEEL_SW_PIN, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_SW_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");
  motor1.setSpeed(64);   // Motor 1 runs forward at 25% speed.
  motor2.setSpeed(64);  // Motor 2 runs backward at 25% speed.
}


// The loop routine runs over and over again forever.
void loop() {
  Serial.println("Left Wheel Switch: " + String(digitalRead(LEFT_WHEEL_SW_PIN)) + "Right Wheel Switch: "+ String(digitalRead(RIGHT_WHEEL_SW_PIN)));
  Serial.println("Left Encoder: " + String(digitalRead(LEFT_ENC_PIN)) + "Right Encoder: "+ String(digitalRead(RIGHT_ENC_PIN)));
}
