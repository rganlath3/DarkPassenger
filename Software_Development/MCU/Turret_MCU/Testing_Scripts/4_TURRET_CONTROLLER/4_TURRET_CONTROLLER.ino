/*
  By Ranil Ganlath
  This is for testing the Cytron MDD3A motor controller to drive the turret gearbox motor using an ESP32.
  Board: ESP32-S3-WROOM-1
  ESP32 19  - Motor Driver PWM M1A Input
  ESP32 20  - Motor Driver PWM M1B Input
  ESP32 GND - Motor Driver GND
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
CytronMD motor1(PWM_PWM, 19, 20);   // PWM 1A = Pin 19, PWM 1B = Pin 20.


void setup() {

}



void loop() {
  //124 is 6 rounds in 2sec
  //95 is 4 rounds in 2 sec
  //157.25

  
  motor1.setSpeed(124);   // This is 1 shot
  delay(400);
  motor1.setSpeed(0);     // Motor 1 stops.
  delay(5 000);
}
