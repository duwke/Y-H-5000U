/*
 * Y&H HSI 5000U / SRNE Modbus Register Map
 *
 * This file documents all known Modbus registers for the inverter.
 * Protocol: Modbus RTU, Function Code 0x03 (Read Holding Registers)
 * Slave ID: 1
 * Baud: 9600, 8N1
 */

#ifndef MODBUS_REGISTERS_H
#define MODBUS_REGISTERS_H

#include <Arduino.h>

// Modbus Communication Settings
#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUD_RATE 9600
#define MODBUS_TIMEOUT_MS 1000
#define MODBUS_INTER_QUERY_DELAY_MS 300

// ==============================================================================
// KNOWN REGISTER BLOCKS
// ==============================================================================

// Battery Block (0x0100 - 0x010E, 15 registers)
#define REG_BATTERY_START 0x0100
#define REG_BATTERY_COUNT 15

// Individual register offsets within battery block (from 0x0100)
#define REG_BATTERY_VOLTAGE_OFFSET 2      // 0x0102: Battery voltage (0.1V)
#define REG_BATTERY_CURRENT_OFFSET 3      // 0x0103: Battery current (0.1A, signed)
#define REG_PV_VOLTAGE_OFFSET 8           // 0x0108: PV voltage (0.1V)
#define REG_PV_CURRENT_OFFSET 9           // 0x0109: PV current (0.1A)

// System Data Block (0x020C - 0x021B, 16 registers) - AC/Grid Parameters
#define REG_SYSTEM_DATA_START 0x020C
#define REG_SYSTEM_DATA_COUNT 16

// Observed values and interpretation:
// AC Input/Grid Parameters
// 0x020D: 0x0206 (518) = 51.8Hz → Grid frequency (scale 0.01)
// 0x020E: 0x3A13 (14867) → Unknown counter/timer (possibly uptime in minutes)
// 0x020F: 0x0000 (0) → Grid status flag (0 = disconnected, 1 = connected)
// 0x0210: 0x0004 (4) → Operating mode/status (4-5 = inverter mode active?)
// 0x0211: 0x0000 (0) → Fault/alarm code (0 = no faults)

// AC Voltage Measurements (scale 0.1V unless noted)
// 0x0212: 0x116D (4461) = 446.1V → PV array open circuit voltage
// 0x0213: 0x04A2 (1186) = 118.6V → AC mains/grid voltage (input to inverter)
#define REG_AC_INPUT_VOLTAGE 0x0213
// 0x0214: 0x00C4 (196) = 19.6V → Unknown (AC input status or secondary voltage?)

// Power/Energy Registers
// 0x0215: 0x1770 (6000) = 6000W → Maximum PV power capability
// 0x0216: 0x04B3 (1203) = 120.3V or 1203W? → Current load power or voltage?
#define REG_LOAD_POWER_ALT 0x0216
// 0x0217: 0x00A6 (166) = 16.6% or 166? → Load percentage
#define REG_LOAD_PERCENTAGE 0x0217
// 0x0218: 0x1770 (6000) = 6000W → PV power rating/max (same as 0x0215)
// 0x0219: 0x0022 (34) = 34°C → Internal temperature
#define REG_TEMPERATURE 0x0219
// 0x021A: 0x0000 (0) → Reserved/unused
// 0x021B: 0x017B (379) = 37.9A → AC mains/grid current (scale 0.1A)
//   Includes both load power AND battery charging current
//   At 118.6V × 37.9A = 4495W total AC input power
#define REG_AC_INPUT_CURRENT 0x021B

// Load Block (0x021C - 0x023E, 35 registers)
#define REG_LOAD_START 0x021C
#define REG_LOAD_COUNT 35

// Individual register offsets within load block (from 0x021C)
#define REG_LOAD_POWER_OFFSET 0           // 0x021C: Load power (W)

// Extended Load Data (0x022B - 0x023E) - Mostly zeros, unknown purpose

// ==============================================================================
// SRNE CONFIGURATION REGISTERS (0xE000+)
// ==============================================================================

// Config Block 1 (0xE000 - 0xE039, 57 registers) - Battery/Charge Settings
#define REG_CONFIG_BATTERY_START 0xE000
#define REG_CONFIG_BATTERY_COUNT 58

// Battery Voltage Settings (scaled by 10, for 48V system)
// 0xE001: 0x0320 (800) = 80.0V → Overvoltage disconnection (60V * 1.33 safety margin?)
// 0xE003: 0x0030 (48)  = 48V → Battery system voltage (parameter [11] equivalent)
// 0xE006: 0x0238 (568) = 56.8V → Boost/Absorption charge voltage (parameter [09])
// 0xE007-0xE009: 0x0228 (552) = 55.2V → Float charge voltage (parameter [11])
// 0xE00A: 0x02B8 (696) = 69.6V? → Equalization charge voltage (parameter [17])?
// 0xE00B: 0x01A4 (420) = 42.0V → Over-discharge voltage (parameter [12])
// 0xE00C: 0x01B0 (432) = 43.2V? → Low voltage alarm point (parameter [14])?
// 0xE00D: 0x0190 (400) = 40.0V → Discharge limit voltage (parameter [15])

// Timing Parameters (minutes)
// 0xE00F: 0x0005 (5)   = 5 seconds → Over-discharge delay (parameter [13])
// 0xE010: 0x001E (30)  = 30 days → Equalization interval (parameter [20])
// 0xE011: 0x0078 (120) = 120 min → Boost charge time (parameter [10])
// 0xE012: 0x0078 (120) = 120 min → Equalization charge time (parameter [18])

// Temperature Settings (signed values)
// 0xE016: 0xFFE2 (-30) = -30°C → Charge lower limit temp
// 0xE017: 0x003C (60)  = 60°C → Charge upper limit temp
// 0xE018: 0x003C (60)  = 60°C → Discharge upper limit temp
// 0xE019: 0xFFE2 (-30) = -30°C → Discharge lower limit temp

// Config Block 2 (0xE020 - 0xE039, 26 registers) - Extended Settings
#define REG_CONFIG_EXT_START 0xE020
#define REG_CONFIG_EXT_COUNT 26

// 0xE021: 0x0000
// 0xE022: 0x0085 = 133 (13.3V?)
// 0xE023: 0x000A = 10
// 0xE024: 0x001E = 30
// 0xE025-0xE039: Mostly zeros

// Config Block 4 (0xE100 - 0xE11D, 29 registers) - Current/Power Limits
#define REG_CONFIG_LIMITS_START 0xE100
#define REG_CONFIG_LIMITS_COUNT 30

// PV Settings
// 0xE114: 0x000A (10) = Reserved for future PV current limit?
// 0xE115: 0x000A (10) = Reserved
// 0xE116: 0x0022 (34) = Reserved for max PV input current (22A per manual)
// 0xE118: 0x0032 (50) = 50A → Max PV charging current (parameter [36])

// BMS Communication Settings
// 0xE119: RS485 communication mode (parameter [32])
// 0xE11A: BMS protocol selection (parameter [33])

// Config Block 5 (0xE200 - 0xE21B, 27 registers) - System/Model Settings
#define REG_CONFIG_SYSTEM_START 0xE200
#define REG_CONFIG_SYSTEM_COUNT 28

// System Configuration
// 0xE200: Device ID/Address (parameter [30])
// 0xE201: Password/Lock status
// 0xE202: 0x029A (666) → Model identifier?
// 0xE203: Password value (mentioned in SRNE docs)

// Operation Modes (matches manual section 4.5)
// 0xE204: Work Priority Mode (parameter [01])
//   0 = SOL (PV priority - uses solar/battery first, switches to mains when PV invalid or battery low)
//   1 = UTI (Mains priority - uses utility first, inverter only when mains invalid)
//   2 = SBU (Inverter priority - uses battery, switches to mains only on under-voltage)
#define REG_WORK_PRIORITY_MODE 0xE204

// 0xE205: 0x0190 (400) = 40.0A → Max AC charging current (parameter [28])
#define REG_MAX_AC_CHARGING_CURRENT 0xE205
// 0xE206: Equalizing charge enable (parameter [16])
// 0xE207: 0x0019 (25) = 25W → Power save threshold (parameter [22])

// Power Settings
// 0xE208: 0x04B0 (1200) = 120.0V → AC output voltage (parameter [38])
// 0xE209: 0x1770 (6000) = 60.00Hz → Output frequency (parameter [02])

// Charge Settings
// 0xE20A: 0x0190 (400) = 40.0A → Max charge current (parameter [07])
#define REG_MAX_CHARGE_CURRENT 0xE20A
// 0xE20B: 0x0001 = AC input range (parameter [03])
//   // 0=APL (wide), 1=UPS (narrow)

// Enable/Disable Flags (0x0001=enabled, 0x0000=disabled)
// 0xE20C: Energy saving mode (parameter [22])
// 0xE20D: Auto restart after overload (parameter [23])
// 0xE20E: Auto restart after over-temp (parameter [24])
// 0xE20F: Buzzer alarm (parameter [25])
// 0xE210: Mode change alert (parameter [26])
// 0xE211: Inverter overload to bypass (parameter [27])

// Charging Mode (parameter [06])
// 0xE212: Energy flow priority - controls how PV, utility, battery, and load interact
//   0 = CSO (PV→Battery→Utility→Load)
//       Maximize battery charge first, then utility supplements, finally power load
//       Use when: Battery health/capacity is top priority
//   1 = CUB (PV→Utility→Battery→Load) [DEFAULT]
//       Balanced: Charge battery while ensuring load stays powered
//       Use when: Want reliable power delivery with battery backup (most residential)
//   2 = SNU (PV→Battery→Load, Solar first, NO utility charging)
//       Solar powers load immediately, excess to battery, minimize grid usage
//       Use when: Off-grid or want maximum solar utilization / minimal grid dependence
//   3 = OSO (PV→Load→Battery→Utility)
//       Direct solar-to-load delivery, battery charges from excess, utility backup only
//       Use when: High-priority loads need immediate solar power / fastest response
#define REG_CHARGING_MODE 0xE212

// Battery Type (parameter [08])
// 0xE213: Battery type selection
//   // Values: USE, SLd, FLd, GEL, L14-L16, N13-N14

// ==============================================================================
// REGISTER PARSING FUNCTIONS
// ==============================================================================

struct ModbusRegister {
  uint16_t address;
  const char* name;
  const char* unit;
  float scale;
  bool isSigned;
};

// Known registers for documentation
const ModbusRegister KNOWN_REGISTERS[] = {
  {0x0102, "Battery Voltage", "V", 0.1, false},
  {0x0103, "Battery Current", "A", 0.1, true},
  {0x0108, "PV Voltage", "V", 0.1, false},
  {0x0109, "PV Current", "A", 0.1, false},
  {0x021C, "Load Power", "W", 1.0, false}
};

const int KNOWN_REGISTER_COUNT = sizeof(KNOWN_REGISTERS) / sizeof(ModbusRegister);

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================

// Get scaled float value from raw register value
inline float getScaledValue(uint16_t rawValue, float scale, bool isSigned) {
  if (isSigned) {
    int16_t signedValue = (int16_t)rawValue;
    return signedValue * scale;
  }
  return rawValue * scale;
}

// Get register name for logging
inline const char* getRegisterName(uint16_t address) {
  for (int i = 0; i < KNOWN_REGISTER_COUNT; i++) {
    if (KNOWN_REGISTERS[i].address == address) {
      return KNOWN_REGISTERS[i].name;
    }
  }
  return "Unknown";
}

// ==============================================================================
// REGISTER DISCOVERY RANGES
// ==============================================================================

struct RegisterRange {
  uint16_t start;
  uint16_t count;
  const char* description;
};

// Probe SRNE configuration ranges
const RegisterRange DISCOVERY_RANGES[] = {
  // Unknown readable registers from previous scan
  {0x020C, 16, "Unknown system data"},
  {0x022B, 20, "Post-load block"},

  // SRNE configuration ranges (typically 0xE000+)
  {0xE000, 30, "Config block 1"},
  {0xE020, 30, "Config block 2"},
  {0xE040, 30, "Config block 3"},
  {0xE100, 30, "Config block 4"},
  {0xE200, 30, "Settings/password"}
};

const int DISCOVERY_RANGE_COUNT = sizeof(DISCOVERY_RANGES) / sizeof(RegisterRange);

// ==============================================================================
// DISCOVERY RESULTS
// ==============================================================================
//
// SCAN COMPLETE - All register ranges tested
//
// READABLE RANGES (Total: ~128 registers):
//   0x0100 - 0x010E (15 registers)  - Battery/PV data [DOCUMENTED]
//   0x020C - 0x021B (16 registers)  - System data [UNKNOWN PURPOSE]
//   0x021C - 0x023E (35 registers)  - Load/output [PARTIALLY DOCUMENTED]
//   0xE000 - 0xE039 (57 registers)  - SRNE Config: Battery/Charge [TO BE MAPPED]
//   0xE020 - 0xE039 (26 registers)  - SRNE Config: Extended [TO BE MAPPED]
//   0xE100 - 0xE11D (29 registers)  - SRNE Config: Limits [TO BE MAPPED]
//   0xE200 - 0xE21B (27 registers)  - SRNE Config: System [TO BE MAPPED]
//
// INVALID RANGES (All return Modbus exception 0x02):
//   0x010F - 0x020B, 0x023F - 0xDFFF, 0xE03A - 0xE0FF, 0xE21C+
//
// NEXT STEPS:
//   1. Make configuration changes via inverter LCD/buttons
//   2. Re-scan config registers (0xE000+) to see which values changed
//   3. Correlate changes with LCD menu items to identify register purposes
//   4. Document writable registers for remote configuration

#endif // MODBUS_REGISTERS_H
