#include "tmp119.h"

#include <math.h>

// ---- Configuration register bit fields (Figure 8-7 / Table 8-5) -------------
//
// The 16-bit config register packs several settings into one number:
//
//   bit: 15    14    13    12    11:10  9:7   6:5   4    3    2       1
//        HIGH  LOW   DATA  EE    MOD    CONV  AVG   T/nA POL  DR/Alert SoftReset
//        ALRT  ALRT  RDY   BUSY
//
// A "mask" has 1s in the bit positions of one field, and a "shift" is how far
// that field sits from bit 0. To change one field without touching the rest:
//   1. value & ~MASK        -> clear that field's bits (~ flips every bit)
//   2. (newValue << SHIFT)  -> slide the new value into the field's position
//   3. OR them together     -> combine
namespace {
constexpr uint16_t CFG_HIGH_ALERT  = 1u << 15;
constexpr uint16_t CFG_LOW_ALERT   = 1u << 14;
constexpr uint16_t CFG_DATA_READY  = 1u << 13;
constexpr uint16_t CFG_EEPROM_BUSY = 1u << 12;

constexpr uint8_t  CFG_MOD_SHIFT  = 10;
constexpr uint16_t CFG_MOD_MASK   = 0b11u << CFG_MOD_SHIFT;    // 0x0C00
constexpr uint8_t  CFG_CONV_SHIFT = 7;
constexpr uint16_t CFG_CONV_MASK  = 0b111u << CFG_CONV_SHIFT;  // 0x0380
constexpr uint8_t  CFG_AVG_SHIFT  = 5;
constexpr uint16_t CFG_AVG_MASK   = 0b11u << CFG_AVG_SHIFT;    // 0x0060
constexpr uint16_t CFG_TNA        = 1u << 4;
constexpr uint16_t CFG_POL        = 1u << 3;
constexpr uint16_t CFG_DR_ALERT   = 1u << 2;

// The bits we're allowed to write: MOD through DR/Alert (bits 11..2).
// Bits 15..12 are read-only flags, bit 1 is Soft_Reset (never set by us) and
// bit 0 is unused.
constexpr uint16_t CFG_WRITABLE_MASK = 0x0FFC;

// EEPROM unlock register (Table 8-9). Bit 14 mirrors EEPROM_Busy, and reading
// this register has no side effects, unlike reading the config register.
constexpr uint16_t EEUL_EEPROM_BUSY = 1u << 14;

// The value the temperature register holds after reset, before the first
// conversion finishes: 0x8000 = -256 C (Section 8.5.2).
constexpr int16_t TEMP_POWER_ON_VALUE = INT16_MIN;

// Replace one field inside `value` (see the explanation at the top).
uint16_t setField(uint16_t value, uint16_t mask, uint8_t shift, uint16_t fieldValue) {
  return (value & ~mask) | ((fieldValue << shift) & mask);
}

uint16_t setBit(uint16_t value, uint16_t bit, bool on) {
  return on ? (value | bit) : (value & ~bit);
}
}  // namespace

const char* tmp119ErrorToString(TMP119Error err) {
  switch (err) {
    case TMP119Error::Ok:                return "OK";
    case TMP119Error::NoResponse:        return "no response (address NACK) - check wiring, power and pull-ups";
    case TMP119Error::BusError:          return "I2C bus error";
    case TMP119Error::WrongDeviceId:     return "device ID mismatch - not a TMP119?";
    case TMP119Error::EepromBusyTimeout: return "EEPROM stayed busy after power-up";
    case TMP119Error::NoDataYet:         return "no conversion finished yet";
  }
  return "unknown error";
}

TMP119::TMP119(uint8_t address, TwoWire& wire)
    : address_(address), wire_(wire), config_(0x0220) {}  // 0x0220 = factory default

TMP119Error TMP119::begin() {
  // The chip needs 1.5 ms after power-up before it talks (Section 7.3.1).
  // The Teensy itself takes far longer than that to boot, but this keeps the
  // driver safe if begin() is ever called right after switching sensor power on.
  delay(2);

  uint16_t id;
  TMP119Error err = readDeviceId(id);
  if (err != TMP119Error::Ok) return err;
  if ((id & DEVICE_ID_MASK) != EXPECTED_DEVICE_ID) return TMP119Error::WrongDeviceId;

  // During power-up the chip copies its saved settings out of EEPROM, and it
  // ignores writes to the config and limit registers until that's done
  // (Section 7.5.1.1). It takes ~1.5 ms; allow plenty of margin.
  err = waitForEepromIdle(50);
  if (err != TMP119Error::Ok) return err;

  // Load the current settings into our copy. This read also clears any stale
  // alert flags, which is what we want at startup.
  uint16_t cfg;
  err = readRegister(TMP119Reg::CONFIG, cfg);
  if (err != TMP119Error::Ok) return err;
  config_ = cfg & CFG_WRITABLE_MASK;
  return TMP119Error::Ok;
}

// ---- Low-level register access ---------------------------------------------
//
// Every TMP119 register is 16 bits, sent most significant byte first (MSB, then LSB).
//
// Read (Figure 7-11):  START, address+W, register number,
//                      repeated START, address+R, MSB, LSB, STOP
// Write (Figure 7-10): START, address+W, register number, MSB, LSB, STOP

TMP119Error TMP119::readRegister(uint8_t reg, uint16_t& value) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  // false = don't send STOP, so the read below uses a repeated START.
  uint8_t result = wire_.endTransmission(false);
  if (result == 2) return TMP119Error::NoResponse;  // 2 = address NACK
  if (result != 0) return TMP119Error::BusError;

  if (wire_.requestFrom(address_, (uint8_t)2) != 2) return TMP119Error::BusError;
  uint8_t msb = wire_.read();
  uint8_t lsb = wire_.read();
  // Glue the two bytes together: MSB goes in the top 8 bits.
  value = ((uint16_t)msb << 8) | lsb;
  return TMP119Error::Ok;
}

TMP119Error TMP119::writeRegister(uint8_t reg, uint16_t value) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.write((uint8_t)(value >> 8));    // MSB: shift the top byte down
  wire_.write((uint8_t)(value & 0xFF));  // LSB: keep only the bottom byte
  uint8_t result = wire_.endTransmission();
  if (result == 2) return TMP119Error::NoResponse;
  if (result != 0) return TMP119Error::BusError;
  return TMP119Error::Ok;
}

TMP119Error TMP119::readDeviceId(uint16_t& raw) {
  return readRegister(TMP119Reg::DEVICE_ID, raw);
}

TMP119Error TMP119::waitForEepromIdle(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (true) {
    uint16_t ul;
    TMP119Error err = readRegister(TMP119Reg::EEPROM_UL, ul);
    if (err != TMP119Error::Ok) return err;
    if ((ul & EEUL_EEPROM_BUSY) == 0) return TMP119Error::Ok;
    if (millis() - start > timeoutMs) return TMP119Error::EepromBusyTimeout;
    delay(1);
  }
}

// ---- Temperature -------------------------------------------------------------

float TMP119::rawToCelsius(int16_t raw) {
  return raw * 0.0078125f;  // = raw / 128
}

int16_t TMP119::celsiusToRaw(float celsius) {
  // The register can hold -256 C (0x8000) up to 255.9921875 C (0x7FFF).
  if (celsius < -256.0f) celsius = -256.0f;
  if (celsius > 255.9921875f) celsius = 255.9921875f;
  // Multiply by 128 and round to the nearest step. Negative values come out
  // in two's complement automatically, e.g. -25 C -> -3200 -> 0xF380 (Table 7-1).
  return (int16_t)lroundf(celsius * 128.0f);
}

TMP119Error TMP119::readTemperatureRaw(int16_t& raw) {
  uint16_t value;
  TMP119Error err = readRegister(TMP119Reg::TEMP_RESULT, value);
  if (err != TMP119Error::Ok) return err;
  // The register is two's complement, so reinterpreting the 16 bits as a
  // signed int16_t gives the right sign: 0xFFFF -> -1, 0xF380 -> -3200.
  raw = (int16_t)value;
  if (raw == TEMP_POWER_ON_VALUE) return TMP119Error::NoDataYet;
  return TMP119Error::Ok;
}

TMP119Error TMP119::readTemperatureC(float& celsius) {
  int16_t raw;
  TMP119Error err = readTemperatureRaw(raw);
  if (err != TMP119Error::Ok) return err;
  celsius = rawToCelsius(raw);
  return TMP119Error::Ok;
}

// ---- Configuration -----------------------------------------------------------

TMP119Error TMP119::writeConfig() {
  return writeRegister(TMP119Reg::CONFIG, config_ & CFG_WRITABLE_MASK);
}

TMP119Error TMP119::setMode(TMP119Mode mode) {
  if (mode == TMP119Mode::OneShot) return triggerOneShot();
  config_ = setField(config_, CFG_MOD_MASK, CFG_MOD_SHIFT, (uint16_t)mode);
  return writeConfig();
}

TMP119Error TMP119::triggerOneShot() {
  config_ = setField(config_, CFG_MOD_MASK, CFG_MOD_SHIFT, (uint16_t)TMP119Mode::OneShot);
  TMP119Error err = writeConfig();
  // After the conversion the chip puts itself into shutdown (Section 7.4.3).
  // Record that in our copy, so a later setter doesn't write MOD=11 again and
  // accidentally start another conversion.
  config_ = setField(config_, CFG_MOD_MASK, CFG_MOD_SHIFT, (uint16_t)TMP119Mode::Shutdown);
  return err;
}

TMP119Error TMP119::setAveraging(TMP119Averaging avg) {
  config_ = setField(config_, CFG_AVG_MASK, CFG_AVG_SHIFT, (uint16_t)avg);
  return writeConfig();
}

TMP119Error TMP119::setConversionCycle(TMP119ConvCycle cycle) {
  config_ = setField(config_, CFG_CONV_MASK, CFG_CONV_SHIFT, (uint16_t)cycle);
  return writeConfig();
}

TMP119Error TMP119::setAlertFunction(TMP119AlertFunction fn) {
  config_ = setBit(config_, CFG_TNA, fn == TMP119AlertFunction::Therm);
  return writeConfig();
}

TMP119Error TMP119::setAlertActiveHigh(bool activeHigh) {
  config_ = setBit(config_, CFG_POL, activeHigh);
  return writeConfig();
}

TMP119Error TMP119::setAlertPinShowsDataReady(bool dataReady) {
  config_ = setBit(config_, CFG_DR_ALERT, dataReady);
  return writeConfig();
}

TMP119Error TMP119::setHighLimitC(float celsius) {
  return writeRegister(TMP119Reg::THIGH_LIMIT, (uint16_t)celsiusToRaw(celsius));
}

TMP119Error TMP119::setLowLimitC(float celsius) {
  return writeRegister(TMP119Reg::TLOW_LIMIT, (uint16_t)celsiusToRaw(celsius));
}

// ---- Status ------------------------------------------------------------------

TMP119Error TMP119::readStatus(TMP119Status& status) {
  uint16_t cfg;
  TMP119Error err = readRegister(TMP119Reg::CONFIG, cfg);
  if (err != TMP119Error::Ok) return err;
  // Each flag is a single bit, so AND with its mask: nonzero means "set".
  status.highAlert  = (cfg & CFG_HIGH_ALERT) != 0;
  status.lowAlert   = (cfg & CFG_LOW_ALERT) != 0;
  status.dataReady  = (cfg & CFG_DATA_READY) != 0;
  status.eepromBusy = (cfg & CFG_EEPROM_BUSY) != 0;
  return TMP119Error::Ok;
}
