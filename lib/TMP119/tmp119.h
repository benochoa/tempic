// Driver for the TI TMP119 digital temperature sensor (I2C).
// All register addresses and bit positions come from datasheet SNIS236 (Jan 2024),
// Section 8.5 "Register Map". Section/table numbers are noted next to each value.
//
// This driver only touches the volatile registers. It never unlocks or writes
// the EEPROM, so nothing it does survives a power cycle.

#pragma once

#include <Arduino.h>
#include <Wire.h>

// ---- Register addresses (Table 8-2) ----------------------------------------
namespace TMP119Reg {
constexpr uint8_t TEMP_RESULT = 0x00;  // read-only, signed temperature
constexpr uint8_t CONFIG      = 0x01;  // configuration + status flags
constexpr uint8_t THIGH_LIMIT = 0x02;
constexpr uint8_t TLOW_LIMIT  = 0x03;
constexpr uint8_t EEPROM_UL   = 0x04;  // EEPROM unlock. We only ever READ this.
constexpr uint8_t TEMP_OFFSET = 0x07;
constexpr uint8_t DEVICE_ID   = 0x0F;
}  // namespace TMP119Reg

// Conversion mode, MOD[1:0] (Table 8-5).
enum class TMP119Mode : uint8_t {
  Continuous = 0b00,
  Shutdown   = 0b01,
  OneShot    = 0b11,  // one conversion, then the chip drops back to shutdown
};

// Averaging, AVG[1:0] (Table 8-5). Each conversion takes ~15.5 ms, so 8x
// averaging means ~124 ms of active conversion per result.
enum class TMP119Averaging : uint8_t {
  None  = 0b00,
  Avg8  = 0b01,  // factory default
  Avg32 = 0b10,
  Avg64 = 0b11,
};

// Conversion cycle time, CONV[2:0] (Table 8-6). This is how often a new result
// appears in continuous mode. Averaging sets a minimum: e.g. with Avg32 the
// cycle is never shorter than 500 ms, and with Avg64 never shorter than 1 s.
enum class TMP119ConvCycle : uint8_t {
  Ms15_5 = 0b000,
  Ms125  = 0b001,
  Ms250  = 0b010,
  Ms500  = 0b011,
  S1     = 0b100,  // factory default
  S4     = 0b101,
  S8     = 0b110,
  S16    = 0b111,
};

// What the limits do (T/nA bit, Section 7.4.4).
enum class TMP119AlertFunction : uint8_t {
  Alert = 0,  // window detector: flags latch until the config register is read
  Therm = 1,  // threshold + hysteresis: ALERT follows the HIGH flag
};

enum class TMP119Error : uint8_t {
  Ok,
  NoResponse,         // nobody ACKed address 0x48 (wiring, power, pull-ups)
  BusError,           // other I2C failure (data NACK, short read, ...)
  WrongDeviceId,      // something answered, but it isn't a TMP11x
  EepromBusyTimeout,  // chip never finished loading its power-on settings
  NoDataYet,          // temperature still reads -256 C (no conversion done yet)
};

const char* tmp119ErrorToString(TMP119Error err);

// Snapshot of the four status flags at the top of the config register.
struct TMP119Status {
  bool highAlert;
  bool lowAlert;
  bool dataReady;
  bool eepromBusy;
};

class TMP119 {
 public:
  // The low 12 bits of the Device ID register. Section 8.5.11 says the full
  // register reads 0x2117 (revision 2), while Table 8-2 says 0x0117. We only
  // compare the ID bits so either value passes. Note: the TMP117 also reports
  // 0x117, so this check can't tell a TMP117 from a TMP119.
  static constexpr uint16_t DEVICE_ID_MASK = 0x0FFF;
  static constexpr uint16_t EXPECTED_DEVICE_ID = 0x117;

  explicit TMP119(uint8_t address = 0x48, TwoWire& wire = Wire);

  // Waits for the chip to finish booting, checks the device ID and loads the
  // current configuration. Call Wire.begin() before this.
  TMP119Error begin();

  // ---- Temperature ----
  // Reads the latest result. Returns NoDataYet if no conversion has finished.
  TMP119Error readTemperatureC(float& celsius);
  TMP119Error readTemperatureRaw(int16_t& raw);

  // ---- Configuration (volatile registers only) ----
  TMP119Error setMode(TMP119Mode mode);
  TMP119Error setAveraging(TMP119Averaging avg);
  TMP119Error setConversionCycle(TMP119ConvCycle cycle);
  TMP119Error setAlertFunction(TMP119AlertFunction fn);
  TMP119Error setAlertActiveHigh(bool activeHigh);       // POL bit
  TMP119Error setAlertPinShowsDataReady(bool dataReady);  // DR/Alert bit

  // Starts a single conversion. Poll readStatus().dataReady (or just wait
  // about 15.5 ms x the averaging count) before reading the result.
  TMP119Error triggerOneShot();

  TMP119Error setHighLimitC(float celsius);
  TMP119Error setLowLimitC(float celsius);

  // ---- Status ----
  // WARNING: reading the config register has side effects. In alert mode it
  // clears HIGH_Alert, LOW_Alert and Data_Ready and releases the ALERT pin.
  TMP119Error readStatus(TMP119Status& status);

  // ---- Low level ----
  TMP119Error readRegister(uint8_t reg, uint16_t& value);
  TMP119Error writeRegister(uint8_t reg, uint16_t value);
  TMP119Error readDeviceId(uint16_t& raw);

  // Unit conversion. 1 LSB = 1/128 C = 0.0078125 C (Section 7.3.3).
  static float rawToCelsius(int16_t raw);
  static int16_t celsiusToRaw(float celsius);  // clamps to -256 .. +255.99 C

 private:
  TMP119Error waitForEepromIdle(uint32_t timeoutMs);
  TMP119Error writeConfig();

  uint8_t address_;
  TwoWire& wire_;

  // Our own copy of the writable config bits. We change this copy and write it
  // out instead of reading the chip's register first, because reading the
  // register would clear the alert flags as a side effect.
  uint16_t config_;
};
