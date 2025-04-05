/*
  By Ranil Ganlath
  This is for testing relays with the ESP32 3V logic.
  ESP32 35  - Relay FWD Headlight Control Output
  ESP32 36  - Relay AFT Brake Light Control Output
  
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

#define FWD_RELAY_CTRL 35
#define AFT_RELAY_CTRL 36

void setup() {
  pinMode(FWD_RELAY_CTRL,OUTPUT);
  pinMode(AFT_RELAY_CTRL,OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(FWD_RELAY_CTRL,HIGH);
  delay(5000);
  digitalWrite(FWD_RELAY_CTRL,LOW);
  delay(5000);
  digitalWrite(AFT_RELAY_CTRL,HIGH);
  delay(5000);
  digitalWrite(AFT_RELAY_CTRL,LOW);
  delay(5000);
  digitalWrite(FWD_RELAY_CTRL,HIGH);
  digitalWrite(AFT_RELAY_CTRL,HIGH);
  delay(5000);
  digitalWrite(FWD_RELAY_CTRL,LOW);
  digitalWrite(AFT_RELAY_CTRL,LOW);
  delay(5000);
}
