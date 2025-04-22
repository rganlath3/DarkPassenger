#include <TinyGPSPlus.h>

// GPS module connection
static const int RXPin = 38, TXPin = 37;
static const uint32_t GPSBaud = 9600;
HardwareSerial ss(2);

// The TinyGPSPlus object
TinyGPSPlus gps;

// MNT timezone offset (UTC-7 for Mountain Standard Time)
// Change to -6 during Daylight Saving Time
const int timeZoneOffset = -6;

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  
  Serial.println(F("GPS Data Logger (MNT Time Zone)"));
  Serial.println(F("Type 'G' to get GPS data (satellites, latitude, longitude, date, time)"));
  Serial.println(F("-----------------------------------------------------------------"));
}

void loop() {
  // Process GPS data continuously
  while (ss.available())
    gps.encode(ss.read());
  
  // Check if user typed "G" in Serial Monitor
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    
    // Print GPS data when 'G' is received
    if (incomingByte == 'G' || incomingByte == 'g') {
      printGPSData();
    }
  }
}

void printGPSData() {
  // Print the essential GPS data
  if (gps.satellites.isValid()) {
    Serial.print(F("Satellites: "));
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("Satellites: *"));
  }
  
  Serial.print(F(" | "));
  
  if (gps.location.isValid()) {
    Serial.print(F("Lat: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" | Lng: "));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("Lat: * | Lng: *"));
  }
  
  Serial.print(F(" | "));
  
  // Date
  if (gps.date.isValid()) {
    Serial.print(F("Date: "));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("Date: *"));
  }
  
  Serial.print(F(" | "));
  
  // Time (adjusted for MNT)
  if (gps.time.isValid()) {
    Serial.print(F("Time (MNT): "));
    
    // Adjust hour for MNT
    int hour = gps.time.hour() + timeZoneOffset;
    
    // Handle day boundary crossing
    if (hour < 0) {
      hour += 24;
    } else if (hour >= 24) {
      hour -= 24;
    }
    
    if (hour < 10) Serial.print(F("0"));
    Serial.print(hour);
    Serial.print(F(":"));
    
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
  } else {
    Serial.print(F("Time (MNT): *"));
  }
  
  Serial.println();
  
  // Check if no GPS data is being received
  if (gps.charsProcessed() < 10)
    Serial.println(F("WARNING: No GPS data received - check wiring"));
}
