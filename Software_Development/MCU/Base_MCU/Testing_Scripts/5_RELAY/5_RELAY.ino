/*
  By Ranil Ganlath
  This is for testing relays with the ESP32 3V logic.
  ESP32 15  - Relay Headlight Control Output
  ESP32 18  - Relay Brake Light Control Output
  
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
  pinMode(15,OUTPUT);
  pinMode(18,OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(15,HIGH);
  delay(5000);
  digitalWrite(15,LOW);
  delay(5000);
  digitalWrite(18,HIGH);
  delay(5000);
  digitalWrite(18,LOW);
  delay(5000);
  digitalWrite(15,HIGH);
  digitalWrite(18,HIGH);
  delay(5000);
  digitalWrite(15,LOW);
  digitalWrite(18,LOW);
  delay(5000);
}
