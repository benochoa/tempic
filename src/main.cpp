// TMP119 temperature monitor for Teensy 4.1.
//
// Wiring (see schematic):
//   TMP119 SDA   -> Teensy pin 18 (SDA, default Wire bus)
//   TMP119 SCL   -> Teensy pin 19 (SCL)
//   TMP119 ALERT -> Teensy pin 2  (open-drain, external 5k pull-up to 3.3 V)
//   TMP119 ADD0  -> GND, so the I2C address is 0x48
//
// Prints the temperature about once per second, and prints a message whenever
// the temperature goes above ALERT_HIGH_C or below ALERT_LOW_C.

#include <Arduino.h>
#include <Wire.h>

#include "tmp119.h"

// ---- Settings you might change ---------------------------------------------
#define ALERT_PIN 2                // Teensy pin wired to TMP119 ALERT
#define TMP119_ADDRESS 0x48        // ADD0 tied to GND (Table 7-2)
#define I2C_CLOCK_HZ 400000        // fast mode; the chip supports up to 400 kHz
#define I2C_FALLBACK_CLOCK_HZ 100000

const float ALERT_HIGH_C = 60.0f;  // placeholder limits
const float ALERT_LOW_C = 0.0f;

const uint32_t PRINT_INTERVAL_MS = 1000;
// -----------------------------------------------------------------------------

TMP119 sensor(TMP119_ADDRESS, Wire);

// Set by the interrupt, cleared by loop(). "volatile" tells the compiler the
// value can change at any moment (from the interrupt), so it must re-read it
// from memory every time instead of caching it.
volatile bool alertFired = false;

// Interrupt service routine: runs the instant ALERT goes low. Keep it tiny.
// No I2C, no Serial, no delays in here; just raise a flag for loop() to handle.
void onAlertFalling() {
  alertFired = true;
}

// Stops here forever and keeps printing the error, so you can't miss it.
void halt(const char* what, TMP119Error err) {
  while (true) {
    Serial.printf("FATAL: %s: %s\n", what, tmp119ErrorToString(err));
    delay(2000);
  }
}

void check(const char* what, TMP119Error err) {
  if (err != TMP119Error::Ok) halt(what, err);
}

void setup() {
  Serial.begin(115200);
  // Teensy's Serial is USB. Give the PC up to 3 s to open the port so you see
  // the startup messages, but don't hang forever if nothing is plugged in.
  while (!Serial && millis() < 3000) {}
  Serial.println("TMP119 monitor starting");

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  TMP119Error err = sensor.begin();
  if (err == TMP119Error::NoResponse) {
    // Long wires or weak pull-ups can make 400 kHz unreliable. Try slower.
    Serial.println("No response at 400 kHz, retrying at 100 kHz");
    Wire.setClock(I2C_FALLBACK_CLOCK_HZ);
    err = sensor.begin();
  }
  if (err != TMP119Error::Ok) {
    uint16_t id = 0;
    if (sensor.readDeviceId(id) == TMP119Error::Ok) {
      Serial.printf("Device ID register read 0x%04X (expected 0x?117)\n", id);
    }
    halt("sensor.begin()", err);
  }

  uint16_t id = 0;
  sensor.readDeviceId(id);
  Serial.printf("Found TMP11x at 0x%02X, device ID register = 0x%04X\n", TMP119_ADDRESS, id);

  // Configure the volatile registers. These are the factory defaults except
  // the limits, set explicitly so the code doesn't depend on what's in EEPROM.
  check("setMode", sensor.setMode(TMP119Mode::Continuous));
  check("setAveraging", sensor.setAveraging(TMP119Averaging::Avg8));
  check("setConversionCycle", sensor.setConversionCycle(TMP119ConvCycle::S1));
  check("setAlertFunction", sensor.setAlertFunction(TMP119AlertFunction::Alert));
  check("setAlertActiveHigh", sensor.setAlertActiveHigh(false));  // active low
  check("setAlertPinShowsDataReady", sensor.setAlertPinShowsDataReady(false));
  check("setHighLimitC", sensor.setHighLimitC(ALERT_HIGH_C));
  check("setLowLimitC", sensor.setLowLimitC(ALERT_LOW_C));
  Serial.printf("Alert limits: low %.2f C, high %.2f C\n", ALERT_LOW_C, ALERT_HIGH_C);

  // ALERT is open-drain with an external pull-up, so plain INPUT (no internal
  // pull-up needed). Interrupt on the high-to-low edge, since it's active low.
  pinMode(ALERT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ALERT_PIN), onAlertFalling, FALLING);

  // If ALERT was already low before the interrupt was attached, there will be
  // no falling edge to catch. Handle it on the first loop() instead.
  if (digitalRead(ALERT_PIN) == LOW) alertFired = true;
}

void handleAlert() {
  // In alert mode this read also clears the flags and releases the ALERT pin.
  TMP119Status status;
  TMP119Error err = sensor.readStatus(status);
  if (err != TMP119Error::Ok) {
    Serial.printf("Alert: failed to read status: %s\n", tmp119ErrorToString(err));
    return;
  }
  if (status.highAlert) Serial.printf("ALERT: temperature above %.2f C\n", ALERT_HIGH_C);
  if (status.lowAlert) Serial.printf("ALERT: temperature below %.2f C\n", ALERT_LOW_C);
  if (!status.highAlert && !status.lowAlert) Serial.println("ALERT pin fired but no flag set");
}

void loop() {
  if (alertFired) {
    // Clear first: if another alert arrives while we're handling this one,
    // the interrupt sets the flag again and we catch it next time round.
    alertFired = false;
    handleAlert();
  }

  // millis() - lastPrint keeps working correctly even when millis() wraps
  // around after ~49 days, because unsigned subtraction wraps too.
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    // TODO: the datasheet recommends avoiding I2C traffic during a conversion
    // for best accuracy (Section 7.5.3.1.1). Reading on a fixed timer can
    // overlap the ~124 ms conversion window. If that matters, poll Data_Ready
    // instead, or switch to one-shot mode.
    float tempC;
    TMP119Error err = sensor.readTemperatureC(tempC);
    if (err == TMP119Error::Ok) {
      Serial.printf("Temperature: %.4f C\n", tempC);
    } else {
      Serial.printf("Temperature read failed: %s\n", tmp119ErrorToString(err));
    }
  }
}
