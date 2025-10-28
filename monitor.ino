/*
 * Y&H HSI 5000U / SRNE Solar Inverter Monitor for Arduino R4
 * Based on SRNE Modbus RTU protocol
 * 
 * Hardware:
 * - RS485 to TTL converter module (auto-direction type with TX/RX only)
 * 
 * Connections:
 * Y&H RJ45 Pin 7 (RS485-A) -> RS485 Module A
 * Y&H RJ45 Pin 8 (RS485-B) -> RS485 Module B
 * Y&H RJ45 Pin 1 (GND) -> RS485 Module GND & Arduino GND
 * 
 * RS485 Module VCC -> Arduino 5V
 * RS485 Module RX -> Arduino Serial1 TX (Pin 1)
 * RS485 Module TX -> Arduino Serial1 RX (Pin 0)
 * 
 * Arduino IoT Cloud Variables Required:
 * - batteryCurrent (CloudElectricCurrent, Read)
 * - pvCurrent (CloudElectricCurrent, Read)
 * - batteryVoltage (CloudElectricPotential, Read)
 * - pvVoltage (CloudElectricPotential, Read)
 * - batterySOC (int, Read)
 * - loadPower (CloudPower, Read)
 * - pvPower (CloudPower, Read)
 * - enableBuzzer (bool, Read & Write) - callback: onEnableBuzzerChange
 * - takeSnapshot (bool, Read & Write) - callback: onTakeSnapshotChange
 * - compareSnapshot (bool, Read & Write) - callback: onCompareSnapshotChange
 * 
 * Control everything via Arduino IoT Cloud dashboard!
 */

#include <Arduino.h>
#include "thingProperties.h"

// Modbus configuration
#define SLAVE_ID 1                 // Y&H uses slave ID 1 (confirmed working)
#define MODBUS_BAUD 9600          // Y&H uses 9600 baud (confirmed working)
// Note: PowMr models use Slave ID 5 and 2400 baud, but those DON'T work on Y&H HSI 5000U

// Register addresses (Y&H HSI 5000U specific)
#define REG_BATTERY_START 0x0100   // Battery & PV data (confirmed working)
#define REG_BATTERY_COUNT 15       

#define REG_LOAD_START 0x021C      // Load/AC output (confirmed working)
#define REG_LOAD_COUNT 15

// Modbus timing
#define MODBUS_TIMEOUT 1000        // 1 second timeout
#define POLL_INTERVAL 5000         // Poll every 5 seconds
#define DEBUG_MODBUS false         // Set to true for detailed Modbus TX/RX logging

// Known write registers (need verification for Y&H HSI 5000U)
#define REG_BUZZER_ALARM 0x138A    // 5002 decimal - Buzzer on/off (unverified)

// Data structure to hold inverter data
struct InverterData {
  float batteryVoltage;
  float batteryCurrent;
  float pvVoltage;
  float pvCurrent;
  float pvPower;
  float loadPower;
  float acInputVoltage;
  float acOutputVoltage;
  int batterySOC;
  uint16_t inverterStatus;
  bool valid;
};

InverterData inverterData;
unsigned long lastPoll = 0;

// Response buffer for parsing
uint8_t responseBuffer[256];
uint8_t responseBytesRead = 0;

// Global variable to store last read start address
uint16_t lastReadAddress = 0;

// Flag to suppress error messages during scanning
bool suppressScanErrors = false;

// Parameter snapshot system for detecting changes
#define MAX_SNAPSHOT_REGS 100
struct ParameterSnapshot {
  uint16_t address;
  uint16_t value;
  bool isVolatile;  // True if this register changes frequently (telemetry)
};
ParameterSnapshot paramSnapshot[MAX_SNAPSHOT_REGS];
int snapshotCount = 0;
bool snapshotTaken = false;

// Snapshot/compare state machine
enum ScanState {
  SCAN_IDLE,
  SCAN_SNAPSHOTTING,
  SCAN_BASELINE,      // Second snapshot to detect volatile registers
  SCAN_COMPARING
};
ScanState scanState = SCAN_IDLE;
int currentRangeIndex = 0;
int currentRegisterIndex = 0;
unsigned long lastScanTime = 0;
#define SCAN_INTERVAL 150  // ms between register reads during scan

// Scan ranges based on what we know works on Y&H HSI 5000U
struct ScanRange {
  uint16_t start;
  uint16_t count;
};
ScanRange scanRanges[] = {
  {0x0000, 50},   // Very low range - might have basic settings
  {0x0050, 30},   // Low range continuation
  {0x00C0, 30},   // Just before battery block (0x0100)
  {0x0110, 30},   // Just after battery block
  {0x0200, 30},   // Just before load block (0x021C)
  {0x0230, 30},   // Just after load block
  {0x0300, 30},   // Next hundred block
  {0x0400, 30}    // Another hundred block
};
const int numScanRanges = 8;

// Forward declarations
uint16_t swapBytes(uint16_t value);
uint16_t calculateCRC(uint8_t* data, uint8_t length);
bool writeModbusRegister(uint16_t address, uint16_t value);
bool readModbusRegisters(uint16_t startAddress, uint16_t numRegisters);
int readSingleRegister(uint16_t address);
void parseInverterData();
void decodeBatteryBlock();
void decodeLoadBlock();
void printInverterData();
void startParameterSnapshot();
void startParameterCompare();
void processScanStep();
void onEnableBuzzerChange();
void onTakeSnapshotChange();
void onCompareSnapshotChange();

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);
  
  // Defined in thingProperties.h
  initProperties();
  
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you'll get.
     The default is 0 (only errors).
     Maximum is 4
  */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("Y&H HSI 5000U Monitor v2.0");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("Modbus RTU @ 9600 baud, Slave ID 1");
  Serial.println("Control via Arduino IoT Cloud");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  
  // Modbus communication on Serial1
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1);
  
  delay(1000);  // Give inverter time to stabilize
}

// Swap bytes for little-endian data (PowMr/SRNE format)
uint16_t swapBytes(uint16_t value) {
  return (value >> 8) | (value << 8);
}

// Calculate Modbus CRC16
uint16_t calculateCRC(uint8_t* data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

// Write Single Modbus register (Function Code 6)
bool writeModbusRegister(uint16_t address, uint16_t value) {
  uint8_t request[8];
  
  // Build Modbus RTU write request
  request[0] = SLAVE_ID;                      // Slave address
  request[1] = 0x06;                          // Function code (Write Single Register)
  request[2] = (address >> 8) & 0xFF;         // Register address high byte
  request[3] = address & 0xFF;                // Register address low byte
  request[4] = (value >> 8) & 0xFF;           // Value high byte
  request[5] = value & 0xFF;                  // Value low byte
  
  // Calculate and append CRC
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;                    // CRC low byte
  request[7] = (crc >> 8) & 0xFF;             // CRC high byte
  
  // Clear input buffer
  while (Serial1.available()) {
    Serial1.read();
  }
  
  // Send request
  Serial1.write(request, 8);
  Serial1.flush();
  
  // Debug: Print request
  if (DEBUG_MODBUS) {
    Serial.print("WRITE TX: ");
    for (int i = 0; i < 8; i++) {
      if (request[i] < 0x10) Serial.print("0");
      Serial.print(request[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // Wait for response
  unsigned long startTime = millis();
  
  while (Serial1.available() < 8 && (millis() - startTime) < MODBUS_TIMEOUT) {
    delay(10);
  }
  
  if (Serial1.available() < 5) {
    Serial.print("ERROR: Timeout waiting for write response (got ");
    Serial.print(Serial1.available());
    Serial.println(" bytes)");
    return false;
  }
  
  // Read response
  uint8_t response[8];
  uint8_t bytesRead = 0;
  
  while (Serial1.available() && bytesRead < 8) {
    response[bytesRead++] = Serial1.read();
  }
  
  // Debug: Print response
  if (DEBUG_MODBUS) {
    Serial.print("WRITE RX: ");
    for (int i = 0; i < bytesRead; i++) {
      if (response[i] < 0x10) Serial.print("0");
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // Verify response
  if (response[0] != SLAVE_ID) {
    Serial.println("ERROR: Wrong slave ID in write response");
    return false;
  }
  
  if (response[1] == 0x86) {  // Exception response (0x06 + 0x80)
    Serial.print("ERROR: Modbus write exception code: 0x");
    Serial.println(response[2], HEX);
    return false;
  }
  
  if (response[1] != 0x06) {
    Serial.print("ERROR: Wrong function code in write response: 0x");
    Serial.println(response[1], HEX);
    return false;
  }
  
  // Verify CRC
  uint16_t receivedCRC = response[bytesRead - 2] | (response[bytesRead - 1] << 8);
  uint16_t calculatedCRC = calculateCRC(response, bytesRead - 2);
  
  if (receivedCRC != calculatedCRC) {
    Serial.print("ERROR: Write response CRC mismatch. Received: 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(", Calculated: 0x");
    Serial.println(calculatedCRC, HEX);
    return false;
  }
  
  Serial.println("✅ SUCCESS: Register written successfully!");
  return true;
}

// Read Modbus holding registers (Function Code 3)
bool readModbusRegisters(uint16_t startAddress, uint16_t numRegisters) {
  lastReadAddress = startAddress;  // Track which registers we're reading
  
  uint8_t request[8];
  
  // Build Modbus RTU request
  request[0] = SLAVE_ID;                           // Slave address
  request[1] = 0x03;                               // Function code (Read Holding Registers)
  request[2] = (startAddress >> 8) & 0xFF;        // Start address high byte
  request[3] = startAddress & 0xFF;               // Start address low byte
  request[4] = (numRegisters >> 8) & 0xFF;        // Number of registers high byte
  request[5] = numRegisters & 0xFF;               // Number of registers low byte
  
  // Calculate and append CRC
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;                        // CRC low byte
  request[7] = (crc >> 8) & 0xFF;                 // CRC high byte
  
  // Clear input buffer
  while (Serial1.available()) {
    Serial1.read();
  }
  
  // Send request
  Serial1.write(request, 8);
  Serial1.flush();
  
  // Debug: Print request
  if (DEBUG_MODBUS) {
    Serial.print("TX: ");
    for (int i = 0; i < 8; i++) {
      if (request[i] < 0x10) Serial.print("0");
      Serial.print(request[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // Wait for response
  unsigned long startTime = millis();
  uint8_t expectedBytes = 5 + (numRegisters * 2);  // Slave ID + Function + Byte Count + Data + CRC
  
  while (Serial1.available() < expectedBytes && (millis() - startTime) < MODBUS_TIMEOUT) {
    delay(10);
  }
  
  if (Serial1.available() < 5) {  // Minimum response size
    if (!suppressScanErrors) {
      Serial.print("ERROR: Timeout waiting for response (got ");
      Serial.print(Serial1.available());
      Serial.println(" bytes)");
    }
    return false;
  }
  
  // Read response
  responseBytesRead = 0;
  
  while (Serial1.available() && responseBytesRead < 256) {
    responseBuffer[responseBytesRead++] = Serial1.read();
  }
  
  // Debug: Print response
  if (DEBUG_MODBUS) {
    Serial.print("RX: ");
    for (int i = 0; i < responseBytesRead; i++) {
      if (responseBuffer[i] < 0x10) Serial.print("0");
      Serial.print(responseBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // Verify response
  if (responseBytesRead < 5) {
    if (!suppressScanErrors) {
      Serial.println("ERROR: Response too short");
    }
    return false;
  }
  
  if (responseBuffer[0] != SLAVE_ID) {
    if (!suppressScanErrors) {
      Serial.print("ERROR: Wrong slave ID in response (expected ");
      Serial.print(SLAVE_ID);
      Serial.print(", got ");
      Serial.print(responseBuffer[0]);
      Serial.println(")");
    }
    return false;
  }
  
  if (responseBuffer[1] == 0x83) {  // Exception response
    if (!suppressScanErrors) {
      Serial.print("ERROR: Modbus exception code: 0x");
      Serial.println(responseBuffer[2], HEX);
    }
    return false;
  }
  
  if (responseBuffer[1] != 0x03) {
    if (!suppressScanErrors) {
      Serial.print("ERROR: Wrong function code in response: 0x");
      Serial.println(responseBuffer[1], HEX);
    }
    return false;
  }
  
  // Verify CRC
  uint16_t receivedCRC = responseBuffer[responseBytesRead - 2] | (responseBuffer[responseBytesRead - 1] << 8);
  uint16_t calculatedCRC = calculateCRC(responseBuffer, responseBytesRead - 2);
  
  if (receivedCRC != calculatedCRC) {
    if (!suppressScanErrors) {
      Serial.print("ERROR: CRC mismatch. Received: 0x");
      Serial.print(receivedCRC, HEX);
      Serial.print(", Calculated: 0x");
      Serial.println(calculatedCRC, HEX);
    }
    return false;
  }
  
  inverterData.valid = true;
  return true;
}

// Read a single register value
int readSingleRegister(uint16_t address) {
  if (!readModbusRegisters(address, 1)) {
    return -1;  // Error
  }
  
  if (responseBytesRead >= 7) {
    uint16_t value = (responseBuffer[3] << 8) | responseBuffer[4];
    return value;
  }
  
  return -1;
}

void loop() {
  ArduinoCloud.update();
  
  unsigned long currentMillis = millis();
  
  // Process snapshot/compare scanning incrementally
  processScanStep();
  
  // Poll inverter at specified interval
  if (currentMillis - lastPoll >= POLL_INTERVAL) {
    lastPoll = currentMillis;
    
    // Read battery & PV data
    if (!readModbusRegisters(REG_BATTERY_START, REG_BATTERY_COUNT)) {
      Serial.println("ERROR: Failed to read battery data");
    } else {
      parseInverterData();
    }
    
    delay(300);
    
    // Read load/AC output data
    if (!readModbusRegisters(REG_LOAD_START, REG_LOAD_COUNT)) {
      Serial.println("ERROR: Failed to read load data");
    } else {
      parseInverterData();
    }
    
    printInverterData();
  }
}

// Parse inverter data from Modbus registers
void parseInverterData() {
  // The response format is:
  // [0] Slave ID
  // [1] Function code
  // [2] Byte count
  // [3-N] Register data (2 bytes per register, high byte first)
  // [N+1, N+2] CRC
  
  if (responseBytesRead < 5) {
    Serial.println("ERROR: Not enough data to parse");
    return;
  }
  
  // Decode based on which register block
  if (lastReadAddress == REG_BATTERY_START) {
    decodeBatteryBlock();
  } else if (lastReadAddress == REG_LOAD_START) {
    decodeLoadBlock();
  }
}

// Decode battery block (0x0100 / 256)
// Y&H HSI 5000U register map (confirmed):
// Register 0: Battery SOC (%)
// Register 1: Battery Voltage (*0.1 = volts)
// Register 2: Battery Current (*0.1 = amps, positive when charging, negative when discharging)
// Register 7: PV Voltage (*0.1 = volts)
// Register 8: PV Current (*0.1 = amps)
// Register 9: PV Power (needs verification)
void decodeBatteryBlock() {
  if (responseBytesRead < 23) return;  // Need at least 10 registers
  
  uint16_t soc = (responseBuffer[3] << 8) | responseBuffer[4];
  uint16_t voltage = (responseBuffer[5] << 8) | responseBuffer[6];
  uint16_t currentRaw = (responseBuffer[7] << 8) | responseBuffer[8];
  int16_t current = (int16_t)currentRaw;
  uint16_t pvVoltage = (responseBuffer[17] << 8) | responseBuffer[18];  // Register 7
  uint16_t pvCurrent = (responseBuffer[19] << 8) | responseBuffer[20];  // Register 8
  
  inverterData.batterySOC = soc;
  inverterData.batteryVoltage = voltage * 0.1;
  inverterData.batteryCurrent = current * 0.1;
  inverterData.pvVoltage = pvVoltage * 0.1;
  inverterData.pvCurrent = pvCurrent * 0.1;
  inverterData.pvPower = inverterData.pvVoltage * inverterData.pvCurrent;
  inverterData.valid = true;
}

// Decode load block (0x021C / 540)
// Register 0: Load Power (watts)
// Register 3: Load Current (*0.1 = amps)
void decodeLoadBlock() {
  if (responseBytesRead < 11) return;
  
  uint16_t loadPower = (responseBuffer[3] << 8) | responseBuffer[4];  // Register 0
  inverterData.loadPower = loadPower;
}

// Print inverter data to serial monitor
void printInverterData() {
  if (!inverterData.valid) {
    Serial.println("⚠️  No valid data");
    return;
  }
  
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  // Solar PV
  if (inverterData.pvVoltage > 0 || inverterData.pvPower > 0) {
    Serial.print("☀️  PV: ");
    Serial.print(inverterData.pvVoltage, 1);
    Serial.print("V × ");
    Serial.print(inverterData.pvCurrent, 1);
    Serial.print("A = ");
    Serial.print(inverterData.pvPower, 0);
    Serial.println("W");
  }
  
  // Battery
  Serial.print("🔋 BAT: ");
  Serial.print(inverterData.batterySOC);
  Serial.print("% @ ");
  Serial.print(inverterData.batteryVoltage, 1);
  Serial.print("V, ");
  Serial.print(abs(inverterData.batteryCurrent), 1);
  Serial.print("A ");
  if (inverterData.batteryCurrent < -0.1) {
    Serial.println("⚡ DISCHARGING");
  } else if (inverterData.batteryCurrent > 0.1) {
    Serial.println("🔌 CHARGING");
  } else {
    Serial.println("💤 IDLE");
  }
  
  // Load
  if (inverterData.loadPower > 0) {
    Serial.print("🏠 LOAD: ");
    Serial.print(inverterData.loadPower, 0);
    Serial.println("W");
  }
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  // Update Arduino Cloud variables
  batteryVoltage = inverterData.batteryVoltage;
  batterySOC = inverterData.batterySOC;
  batteryCurrent = inverterData.batteryCurrent;
  pvVoltage = inverterData.pvVoltage;
  pvCurrent = inverterData.pvCurrent;
  pvPower = inverterData.pvPower;
  loadPower = inverterData.loadPower;
}

// Start parameter snapshot
void startParameterSnapshot() {
  if (scanState != SCAN_IDLE) {
    Serial.println("\n⚠️  Scan already in progress!");
    return;
  }
  
  Serial.println("\n========================================");
  Serial.println("📸 STARTING PARAMETER SNAPSHOT");
  Serial.println("========================================");
  Serial.println("Step 1: Scanning Y&H registers (~90 sec)");
  Serial.println("Step 2: Re-scan to detect telemetry (~15 sec)");
  Serial.println("Normal polling continues during scan.");
  Serial.println("========================================\n");
  
  scanState = SCAN_SNAPSHOTTING;
  currentRangeIndex = 0;
  currentRegisterIndex = 0;
  snapshotCount = 0;
  suppressScanErrors = true;
  lastScanTime = millis();
  
  // Initialize isVolatile flags
  for (int i = 0; i < MAX_SNAPSHOT_REGS; i++) {
    paramSnapshot[i].isVolatile = false;
  }
}

// Start parameter comparison
void startParameterCompare() {
  if (scanState != SCAN_IDLE) {
    Serial.println("\n⚠️  Scan already in progress!");
    return;
  }
  
  if (!snapshotTaken) {
    Serial.println("\n❌ ERROR: No snapshot taken yet!");
    Serial.println("Take a snapshot first by sending 'T'\n");
    return;
  }
  
  Serial.println("\n========================================");
  Serial.println("🔍 COMPARING PARAMETERS TO SNAPSHOT");
  Serial.println("========================================");
  Serial.println("Re-reading all snapshot registers...");
  Serial.println("This will take ~15 seconds.");
  Serial.println("========================================\n");
  
  scanState = SCAN_COMPARING;
  currentRangeIndex = 0;
  suppressScanErrors = true;
  lastScanTime = millis();
}

// Process one step of scanning (called from loop)
void processScanStep() {
  if (scanState == SCAN_IDLE) return;
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastScanTime < SCAN_INTERVAL) return;
  lastScanTime = currentMillis;
  
  if (scanState == SCAN_SNAPSHOTTING) {
    // Snapshot mode: scan through all ranges
    if (currentRangeIndex >= numScanRanges) {
      // Done with first snapshot, now do baseline check
      Serial.println("\n========================================");
      Serial.println("✅ Step 1 complete: ");
      Serial.print(snapshotCount);
      Serial.println(" registers found");
      Serial.println("========================================");
      Serial.println("Step 2: Detecting telemetry registers...");
      Serial.println("(waiting 5 seconds for values to change)");
      Serial.println("========================================\n");
      
      scanState = SCAN_BASELINE;
      currentRangeIndex = 0;
      lastScanTime = millis() + 5000;  // Wait 5 seconds before baseline
      return;
    }
    
    ScanRange* range = &scanRanges[currentRangeIndex];
    
    // Print range header on first register
    if (currentRegisterIndex == 0) {
      Serial.print("Range 0x");
      if (range->start < 0x1000) Serial.print("0");
      if (range->start < 0x100) Serial.print("0");
      if (range->start < 0x10) Serial.print("0");
      Serial.print(range->start, HEX);
      Serial.print("-0x");
      uint16_t endAddr = range->start + range->count - 1;
      if (endAddr < 0x1000) Serial.print("0");
      if (endAddr < 0x100) Serial.print("0");
      if (endAddr < 0x10) Serial.print("0");
      Serial.print(endAddr, HEX);
      Serial.print(": ");
    }
    
    // Try to read one register
    uint16_t addr = range->start + currentRegisterIndex;
    int value = readSingleRegister(addr);
    
    if (value >= 0 && snapshotCount < MAX_SNAPSHOT_REGS) {
      // Found a register!
      paramSnapshot[snapshotCount].address = addr;
      paramSnapshot[snapshotCount].value = value;
      snapshotCount++;
      
      if (currentRegisterIndex == 0) Serial.println();  // Newline after range header
      Serial.print("  ✓ 0x");
      if (addr < 0x1000) Serial.print("0");
      if (addr < 0x100) Serial.print("0");
      if (addr < 0x10) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" = ");
      Serial.println(value);
    }
    
    // Move to next register
    currentRegisterIndex++;
    if (currentRegisterIndex >= range->count) {
      // Done with this range, check if we found nothing
      if (currentRegisterIndex == range->count) {
        bool foundAny = false;
        for (int i = 0; i < snapshotCount; i++) {
          if (paramSnapshot[i].address >= range->start && 
              paramSnapshot[i].address < range->start + range->count) {
            foundAny = true;
            break;
          }
        }
        if (!foundAny && currentRegisterIndex > 0) {
          Serial.println("none");
        }
      }
      currentRangeIndex++;
      currentRegisterIndex = 0;
    }
    
  } else if (scanState == SCAN_BASELINE) {
    // Baseline mode: re-read all registers to detect which are volatile (telemetry)
    if (currentRangeIndex >= snapshotCount) {
      // Done with baseline, count how many are stable
      int stableCount = 0;
      int volatileCount = 0;
      for (int i = 0; i < snapshotCount; i++) {
        if (paramSnapshot[i].isVolatile) {
          volatileCount++;
        } else {
          stableCount++;
        }
      }
      
      scanState = SCAN_IDLE;
      suppressScanErrors = false;
      snapshotTaken = true;
      
      Serial.println("\n========================================");
      Serial.println("✅ Snapshot complete!");
      Serial.print("   Total registers: ");
      Serial.println(snapshotCount);
      Serial.print("   Stable (settings): ");
      Serial.println(stableCount);
      Serial.print("   Volatile (telemetry): ");
      Serial.print(volatileCount);
      Serial.println(" [ignored]");
      Serial.println("========================================");
      Serial.println("NOW: Change a setting on your inverter");
      Serial.println("THEN: Send 'C' to compare");
      Serial.println("========================================\n");
      return;
    }
    
    // Re-read one register from snapshot
    uint16_t addr = paramSnapshot[currentRangeIndex].address;
    uint16_t oldValue = paramSnapshot[currentRangeIndex].value;
    int newValue = readSingleRegister(addr);
    
    if (newValue >= 0 && newValue != oldValue) {
      // This register changed - it's telemetry, mark it as volatile
      paramSnapshot[currentRangeIndex].isVolatile = true;
      if (DEBUG_MODBUS) {
        Serial.print("  ⚡ 0x");
        if (addr < 0x1000) Serial.print("0");
        if (addr < 0x100) Serial.print("0");
        if (addr < 0x10) Serial.print("0");
        Serial.print(addr, HEX);
        Serial.println(" = telemetry");
      }
    }
    
    currentRangeIndex++;
    
  } else if (scanState == SCAN_COMPARING) {
    // Compare mode: check snapshot registers for changes (skip volatile ones)
    if (currentRangeIndex >= snapshotCount) {
      // Done comparing
      scanState = SCAN_IDLE;
      suppressScanErrors = false;
      
      Serial.println("\n========================================");
      Serial.println("✅ Comparison complete!");
      Serial.println("========================================\n");
      return;
    }
    
    // Skip volatile (telemetry) registers
    if (paramSnapshot[currentRangeIndex].isVolatile) {
      currentRangeIndex++;
      return;
    }
    
    // Read one stable register from snapshot
    uint16_t addr = paramSnapshot[currentRangeIndex].address;
    uint16_t oldValue = paramSnapshot[currentRangeIndex].value;
    int newValue = readSingleRegister(addr);
    
    if (newValue >= 0 && newValue != oldValue) {
      // Found a change in a stable register - this is a real settings change!
      Serial.println("🎯 SETTING CHANGED!");
      Serial.print("   Address: 0x");
      if (addr < 0x1000) Serial.print("0");
      if (addr < 0x100) Serial.print("0");
      if (addr < 0x10) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" (");
      Serial.print(addr);
      Serial.println(")");
      Serial.print("   Old: ");
      Serial.print(oldValue);
      Serial.print(" → New: ");
      Serial.println(newValue);
      Serial.println("   ⭐ WRITABLE SETTING REGISTER!\n");
    }
    
    currentRangeIndex++;
  }
}

// Cloud callback: Buzzer control
void onEnableBuzzerChange() {
  Serial.println("\n========================================");
  Serial.println("🔔 BUZZER CONTROL FROM CLOUD");
  Serial.println("========================================");
  
  // The enableBuzzer variable contains the desired state
  uint16_t desiredState = enableBuzzer ? 1 : 0;
  
  Serial.print("Cloud requested buzzer: ");
  Serial.println(enableBuzzer ? "ON" : "OFF");
  Serial.println("\nAttempting to write to register 0x138A...");
  
  if (writeModbusRegister(REG_BUZZER_ALARM, desiredState)) {
    Serial.println("✅ Buzzer command sent successfully!");
    Serial.println("🔊 Listen for a beep (or silence)!");
    
    // Verify the write
    delay(500);
    int verifyState = readSingleRegister(REG_BUZZER_ALARM);
    if (verifyState == desiredState) {
      Serial.println("✓ Verified: Register updated correctly!");
    } else if (verifyState >= 0) {
      Serial.print("⚠️  Register value: ");
      Serial.print(verifyState);
      Serial.println(" (unexpected)");
    }
  } else {
    Serial.println("❌ Failed to write buzzer register");
    Serial.println("Register 0x138A may not exist or be read-only");
    Serial.println("Use snapshot/compare feature to find the real buzzer register!");
  }
  
  Serial.println("========================================\n");
}

// Cloud callback: Take snapshot
void onTakeSnapshotChange() {
  if (takeSnapshot) {
    startParameterSnapshot();
    takeSnapshot = false;  // Reset the trigger
  }
}

// Cloud callback: Compare snapshot
void onCompareSnapshotChange() {
  if (compareSnapshot) {
    startParameterCompare();
    compareSnapshot = false;  // Reset the trigger
  }
}
