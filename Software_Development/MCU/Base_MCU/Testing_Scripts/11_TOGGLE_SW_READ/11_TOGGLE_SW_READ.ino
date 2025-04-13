/*
  By Ranil Ganlath
  This is for testing reading the state of toggle switches with the ESP32 3V logic.
  ESP32 47  - Toggle Switch 1
  ESP32 48  - Toggle Switch 2
  
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

#define TOGGLE_SW1_INPUT_PIN 47
#define TOGGLE_SW2_INPUT_PIN 48

int toggleSwitch1_state = 1;        // value read from the SW1
int toggleSwitch2_state = 1;        // value read from the SW2


void setup() {
  Serial.begin(115200);
  pinMode(TOGGLE_SW1_INPUT_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_SW2_INPUT_PIN, INPUT_PULLUP); 
}

// the loop function runs over and over again forever
void loop() {
  toggleSwitch1_state = digitalRead(TOGGLE_SW1_INPUT_PIN);
  toggleSwitch2_state = digitalRead(TOGGLE_SW2_INPUT_PIN);
  // print out the values you read:
  Serial.printf("Toggle Switch 1 Value = %d\n", toggleSwitch1_state);
  Serial.printf("Toggle Switch 2 Value = %d\n", toggleSwitch2_state);

  delay(500);  // delay in between reads for clear read from serial

}
