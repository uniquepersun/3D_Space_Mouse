// testing if all the sensors are getting detected-->
#include <Arduino.h>
#include <Wire.h>

void setup() {
  // Initialize Serial for the Serial Monitor
  Serial.begin(115200);
  
  // Wait up to 3 seconds for the Serial connection to establish (useful for native USB)
  uint32_t start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(10);
  }

  // Map custom I2C pins to match your MicroPython setup
  Wire.setSDA(4);
  Wire.setSCL(5);
  
  // Initialize the I2C bus and set frequency to 400kHz
  Wire.begin();
  Wire.setClock(400000);
}

void loop() {
  byte error, address;
  int count = 0;
  String found_addrs = "";

  // I2C addresses range from 1 to 127
  for(address = 1; address < 127; address++ ) {
    
    // The i2c_scanner uses the return value of Wire.endTransmission 
    // to see if a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      if (count > 0) {
        found_addrs += ", ";
      }
      found_addrs += "0x";
      if (address < 16) found_addrs += "0";
      found_addrs += String(address, HEX);
      
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No devices found");
  } else {
    Serial.printf("%d device(s) found: %s\n", count, found_addrs.c_str());
  }

  delay(1000); // Wait 1 second before scanning again
}
