/*
  By Ranil Ganlath
  This is for testing the laser diode with the ESP32 3V logic.
  ESP32 16  - Laser Diode Control Output
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


void setup() {
  pinMode(16,OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(16,HIGH);
  delay(5000);
  digitalWrite(16,LOW);
  delay(5000);
}
