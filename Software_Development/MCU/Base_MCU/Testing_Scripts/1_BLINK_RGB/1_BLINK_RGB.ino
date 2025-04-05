/*
  By Ranil Ganlath
  This verifies that the ESP32 is being written correctly. It should flash an external neopixel connected to pin 37.
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
//Onboard NEOPIXEL is wired to pin 48

#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)
#define RGBPIN 48 //pin 42 is neopixel breakout pin.

void setup() {
  pinMode(RGBPIN,OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  rgbLedWrite(RGBPIN, RGB_BRIGHTNESS, 0, 0);  // Red
  delay(1000);
  rgbLedWrite(RGBPIN, 0, RGB_BRIGHTNESS, 0);  // Green
  delay(1000);
  rgbLedWrite(RGBPIN, 0, 0, RGB_BRIGHTNESS);  // Blue
  delay(1000);
  rgbLedWrite(RGBPIN, 0, 0, 0);  // Off / black
  delay(1000);
}
