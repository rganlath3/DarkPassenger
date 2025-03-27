/*
  By Ranil Ganlath
  This is for testing 2x stepper motors for the pan tilt function on the turret using an ESP32.
  Board: ESP32-S3-WROOM-1

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

#include <AccelStepper.h>
#include <MultiStepper.h>

int stepPin = 4;
int dirPin = 3;



AccelStepper motor1(1, stepPin, dirPin);

void setup() {
  // put your setup code here, to run once:
  
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  
  
  motor1.setMaxSpeed(1000);
}

void loop() {
  // put your main code here, to run repeatedly:


  motor1.setSpeed(800);
  motor1.runSpeed();
}
