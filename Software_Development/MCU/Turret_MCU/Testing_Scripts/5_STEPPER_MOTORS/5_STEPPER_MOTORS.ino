/*
  By Ranil Ganlath
  This is for testing 2x stepper motors for the pan tilt function on the turret using an ESP32.
  Board: ESP32-S3-WROOM-1
  Stepper 1:
  ESP32 13  - STEP
  ESP32 12  - DIR
  ESP32 11  - CLK
  ESP32 18  - PDN
  ESP32 15  - TX
  ESP32 17  - RX
  ESP32 5  - EN
  ESP32 7  - INDEX
  ESP32 6  - DIAG
  Stepper 2:
  ESP32 37  - STEP
  ESP32 36  - DIR
  ESP32 38  - CLK
  ESP32 39  - PDN
  ESP32 41  - TX
  ESP32 40  - RX
  ESP32 1  - EN
  ESP32 42  - INDEX
  ESP32 2 - DIAG
  
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
#include <TMC2209.h>
// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

// Hardware serial ports for each stepper driver
HardwareSerial & serial_stream1 = Serial1;
HardwareSerial & serial_stream2 = Serial2;

const long SERIAL_BAUD_RATE = 115200;

// Pin definitions for Stepper 1
const int RX1_PIN = 40;
const int TX1_PIN = 41;
const int EN1_PIN = 1;

// Pin definitions for Stepper 2
const int RX2_PIN = 17;
const int TX2_PIN = 15;
const int EN2_PIN = 5;

const int DELAY = 2000;
const int32_t VELOCITY = 20000;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 80;

// Instantiate two TMC2209 drivers
TMC2209 stepper_driver1;
TMC2209 stepper_driver2;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Setup Stepper 1
  pinMode(EN1_PIN, OUTPUT);
  digitalWrite(EN1_PIN, LOW);
  stepper_driver1.setup(serial_stream1, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, RX1_PIN, TX1_PIN);
  stepper_driver1.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver1.enableCoolStep();
  stepper_driver1.enable();
  stepper_driver1.moveAtVelocity(VELOCITY);
  
  // Setup Stepper 2
  pinMode(EN2_PIN, OUTPUT);
  digitalWrite(EN2_PIN, LOW);
  stepper_driver2.setup(serial_stream2, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, RX2_PIN, TX2_PIN);
  stepper_driver2.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver2.enableCoolStep();
  stepper_driver2.enable();
  stepper_driver2.moveAtVelocity(VELOCITY);
  
  delay(DELAY); // Give time for everything to initialize
}

void loop()
{
  // Check and display status for Stepper 1
  Serial.println("--- Stepper 1 Status ---");
  checkStepperStatus(stepper_driver1, 1);
  
  // Check and display status for Stepper 2
  Serial.println("--- Stepper 2 Status ---");
  checkStepperStatus(stepper_driver2, 2);
  
  Serial.println();
  delay(DELAY);
}

// Function to check and display stepper status
void checkStepperStatus(TMC2209 &stepper, int stepper_num)
{
  if (not stepper.isSetupAndCommunicating())
  {
    Serial.print("Stepper ");
    Serial.print(stepper_num);
    Serial.println(" driver not setup and communicating!");
    return;
  }
  
  bool hardware_disabled = stepper.hardwareDisabled();
  TMC2209::Settings settings = stepper.getSettings();
  TMC2209::Status status = stepper.getStatus();
  
  if (hardware_disabled)
  {
    Serial.print("Stepper ");
    Serial.print(stepper_num);
    Serial.println(" driver is hardware disabled!");
  }
  else if (not settings.software_enabled)
  {
    Serial.print("Stepper ");
    Serial.print(stepper_num);
    Serial.println(" driver is software disabled!");
  }
  else if ((not status.standstill))
  {
    Serial.print("Stepper ");
    Serial.print(stepper_num);
    Serial.print(" moving at velocity ");
    Serial.println(VELOCITY);
    
    uint32_t interstep_duration = stepper.getInterstepDuration();
    Serial.print("which is equal to an interstep_duration of ");
    Serial.println(interstep_duration);
  }
  else
  {
    Serial.print("Stepper ");
    Serial.print(stepper_num);
    Serial.println(" not moving, something is wrong!");
  }
}
