/*
  By Ranil Ganlath
  This is for testing reading the external voltage shifted battery input with the ESP32 3V logic.
  ESP32 1  - Battery Input Pin (from 0 to 3.3V)
  
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

#define BATTERY_INPUT_PIN 1

int batteryVoltage = 3000; // actual battery voltage in volts
int batteryPercentage = 100;

void setup() {
  Serial.begin(115200);
  //set the resolution to 12 bits (0-4095)
  pinMode(BATTERY_INPUT_PIN, INPUT);
  analogReadResolution(12);
}


// the loop function runs over and over again forever
void loop() {
  batteryVoltage = analogReadMilliVolts(BATTERY_INPUT_PIN);
  batteryPercentage = map(batteryVoltage, 2570, 3050, 0, 100);
  if(batteryPercentage>100){batteryPercentage=100;}
  if(batteryPercentage<0){batteryPercentage=0;} 
  Serial.printf("Battery voltage = %d V\n", batteryVoltage);
  Serial.printf("Battery Percentage = %d Percent\n", batteryPercentage);

  if(batteryPercentage<20){
    Serial.println("Warning Battery Low!");
  }
  delay(1000);  // delay in between reads for clear read from serial
}
