/*
  By Ranil Ganlath
  This is for testing relays with the ESP32 3V logic.
  ESP32 4  - Relay Gun LED Control Output
  ESP32 16 - Relay Laser Diode Control Output
  ESP32 10 - Relay Fan Control Output
  
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

#define LED_RELAY_CTRL 4
#define LASER_RELAY_CTRL 16
#define FAN_RELAY_CTRL 10

void setup() {
  pinMode(LED_RELAY_CTRL,OUTPUT);
  pinMode(LASER_RELAY_CTRL,OUTPUT);
  pinMode(FAN_RELAY_CTRL,OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_RELAY_CTRL,HIGH);
  delay(5000);
  digitalWrite(LED_RELAY_CTRL,LOW);
  delay(5000);
  digitalWrite(LASER_RELAY_CTRL,HIGH);
  delay(5000);
  digitalWrite(LASER_RELAY_CTRL,LOW);
  delay(5000);
  digitalWrite(FAN_RELAY_CTRL,HIGH);
  delay(5000);
  digitalWrite(FAN_RELAY_CTRL,LOW);
  delay(5000);
}
