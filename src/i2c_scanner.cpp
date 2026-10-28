// I2C bus scanner. Run this first on new hardware: it tries every address on
// the default Wire bus (SDA pin 18, SCL pin 19) and prints the ones that answer.
// With the TMP119's ADD0 tied to GND you should see exactly one device: 0x48.

#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(100000);  // slow and forgiving for a first test
}

void loop() {
  Serial.println("Scanning I2C bus...");
  int found = 0;
  // Addresses 0x00-0x07 and 0x78-0x7F are reserved by the I2C spec.
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    // An empty transmission: just the address. If a device ACKs, it's there.
    Wire.beginTransmission(addr);
    uint8_t result = Wire.endTransmission();
    if (result == 0) {
      Serial.printf("  Device found at 0x%02X\n", addr);
      found++;
    } else if (result != 2) {  // 2 = no ACK, which is normal for empty addresses
      Serial.printf("  Error %u at 0x%02X\n", result, addr);
    }
  }
  if (found == 0) {
    Serial.println("  No devices found. Check power, SDA/SCL wiring and pull-ups.");
  }
  Serial.println();
  delay(3000);
}
