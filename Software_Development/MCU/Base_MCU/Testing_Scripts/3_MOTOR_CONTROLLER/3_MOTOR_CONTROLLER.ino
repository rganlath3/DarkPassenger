/*
  By Ranil Ganlath
  This is for testing the Cytron MDD10A with 2x motors. With the ESP32, any GPIO pin can be configured as the DIR and PWM pins.
  Board: ESP32-S3-WROOM-1
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

 #include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 11, 13);  // PWM 1 = Pin 11, DIR 1 = Pin 13.
CytronMD motor2(PWM_DIR, 21, 19); // PWM 2 = Pin 21, DIR 2 = Pin 19.

void setup() {
  
}


// The loop routine runs over and over again forever.
void loop() {
  motor1.setSpeed(64);   // Motor 1 runs forward at 25% speed.
  motor2.setSpeed(-64);  // Motor 2 runs backward at 25% speed.
  delay(4000);
  
  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(2000);

  motor1.setSpeed(-64);  // Motor 1 runs backward at 25% speed.
  motor2.setSpeed(64);   // Motor 2 runs forward at 25% speed.
  delay(4000);
  
  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(2000);

  motor1.setSpeed(64);  // Motor 1 runs backward at 25% speed.
  motor2.setSpeed(64);   // Motor 2 runs forward at 25% speed.
  delay(4000);

  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(2000);

  motor1.setSpeed(-64);  // Motor 1 runs backward at 25% speed.
  motor2.setSpeed(-64);   // Motor 2 runs forward at 25% speed.
  delay(4000);

  motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  delay(4000);
}
