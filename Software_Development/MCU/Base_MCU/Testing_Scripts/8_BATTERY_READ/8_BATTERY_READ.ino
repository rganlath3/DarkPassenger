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

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)


void setup() {
  Serial.begin(115200);
  //set the resolution to 12 bits (0-4095)
  analogReadResolution(BATTERY_INPUT_PIN);
}

// the loop function runs over and over again forever
void loop() {
  int analogValue = analogRead(BATTERY_INPUT_PIN);
  int analogVolts = analogReadMilliVolts(BATTERY_INPUT_PIN);

  // print out the values you read:
  Serial.printf("ADC analog value = %d\n", analogValue);
  Serial.printf("ADC millivolts value = %d\n", analogVolts);

  delay(100);  // delay in between reads for clear read from serial
  //outputValue = map(sensorValue, 0, 1023, 0, 255);
}
