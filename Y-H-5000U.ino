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
 * - loadPower (CloudPower, Read)
 * - pvPower (CloudPower, Read)
 * - takeSnapshot (bool, Read & Write) - callback: onTakeSnapshotChange
 * - compareSnapshot (bool, Read & Write) - callback: onCompareSnapshotChange
 * 
 * Control everything via Arduino IoT Cloud dashboard!
 */


#include <Arduino.h>
#include "thingProperties.h"
#include <WiFiS3.h>

// Web server on port 80
WiFiServer server(80);

// Circular buffer for logs
#define MAX_LOG_ENTRIES 100
String logBuffer[MAX_LOG_ENTRIES];
int logIndex = 0;
int logCount = 0;

void logToWeb(String message) {
  // Add timestamp
  String timestampedMsg = String(millis()/1000) + "s: " + message;

  // Store in circular buffer
  logBuffer[logIndex] = timestampedMsg;
  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES) {
    logCount++;
  }

  // Also print to Serial
  Serial.println(timestampedMsg);
}

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
  uint16_t inverterStatus;
  bool valid;
};

InverterData inverterData;
unsigned long lastPoll = 0;
bool deviceIPSet = false;  // Flag to track if deviceIP has been set

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

// Buffer for batching register discoveries before sending to webhook
String registerLogBuffer = "";

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
void queryDeviceIdentification();
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
void handleWebServer();

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);
  
  pinMode(A0, INPUT);  

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
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

  // Wait for Arduino Cloud connection before using webhook
  Serial.print("Waiting for Arduino Cloud connection");
  int cloudAttempts = 0;
  while (ArduinoCloud.connected() == 0 && cloudAttempts < 60) {
    ArduinoCloud.update();  // Process connection
    delay(500);
    Serial.print(".");
    cloudAttempts++;
  }

  if (ArduinoCloud.connected()) {
    Serial.println("\n✅ Arduino Cloud connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Start web server
    server.begin();
    Serial.println("✅ Web server started on port 80");
    Serial.print("📊 View logs at: http://");
    Serial.println(WiFi.localIP());

    logToWeb("Y&H HSI 5000U Monitor v2.0 - Starting up");
  } else {
    Serial.println("\n⚠️  Arduino Cloud not connected, web server disabled");
  }

  // Modbus communication on Serial1
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1);

  delay(1000);  // Give inverter time to stabilize

  // Query device identification on startup
  queryDeviceIdentification();
}

// Measure current with ACS712
// and print on Serial Monitor
const int nSamples = 20;
const float vcc = -12.0;
const int adcMax = 1023;

//const float sens = 0.185;  // 5A
//const float sens = 0.100;  // 20A
const float sens = 0.66;  // 30A

float avg() {
  float val = 0;
  for (int i = 0; i < nSamples; i++) {
    val += analogRead(A0);
    delay(1);
  }
  return val / adcMax / nSamples;
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

// Query Modbus Device Identification (Function Code 43)
void queryDeviceIdentification() {
  String logMsg = "\n========================================\n";
  logMsg += "🔍 QUERYING MODBUS DEVICE IDENTIFICATION\n";
  logMsg += "========================================\n";
  logMsg += "Attempting Modbus Function Code 43 (0x2B)\n";
  logMsg += "MEI Type: Read Device Identification (0x0E)\n\n";

  Serial.print(logMsg);
  logToWeb(logMsg);

  // Try reading Basic Device Identification (Read Device ID Code = 01)
  uint8_t request[8];
  request[0] = SLAVE_ID;           // Slave address
  request[1] = 0x2B;                // Function code 43 (Encapsulated Interface Transport)
  request[2] = 0x0E;                // MEI Type (Read Device Identification)
  request[3] = 0x01;                // Read Device ID code (01 = Basic identification)
  request[4] = 0x00;                // Object ID to start from (00 = VendorName)

  // Calculate and append CRC
  uint16_t crc = calculateCRC(request, 5);
  request[5] = crc & 0xFF;          // CRC low byte
  request[6] = (crc >> 8) & 0xFF;   // CRC high byte

  // Clear input buffer
  while (Serial1.available()) {
    Serial1.read();
  }

  // Send request
  Serial1.write(request, 7);
  Serial1.flush();

  logMsg = "Request sent: ";
  for (int i = 0; i < 7; i++) {
    if (request[i] < 0x10) logMsg += "0";
    logMsg += String(request[i], HEX);
    logMsg += " ";
  }
  logMsg += "\n\n";
  Serial.print(logMsg);
  logToWeb(logMsg);

  // Wait for response
  unsigned long startTime = millis();
  while (Serial1.available() < 5 && (millis() - startTime) < MODBUS_TIMEOUT) {
    delay(10);
  }

  if (Serial1.available() < 5) {
    logMsg = "❌ No response from device\n";
    logMsg += "This device likely doesn't support\n";
    logMsg += "Modbus Device Identification (Function Code 43)\n";
    logMsg += "========================================\n";
    Serial.print(logMsg);
    logToWeb(logMsg);
    return;
  }

  // Read response
  uint8_t response[256];
  uint8_t bytesRead = 0;

  while (Serial1.available() && bytesRead < 256) {
    response[bytesRead++] = Serial1.read();
    delay(1);  // Small delay to allow more bytes to arrive
  }

  logMsg = "Response received (" + String(bytesRead) + " bytes): ";
  for (int i = 0; i < bytesRead; i++) {
    if (response[i] < 0x10) logMsg += "0";
    logMsg += String(response[i], HEX);
    logMsg += " ";
  }
  logMsg += "\n\n";
  Serial.print(logMsg);
  logToWeb(logMsg);

  // Check for exception response
  if (response[1] == 0xAB) {  // Exception: 0x2B + 0x80
    logMsg = "❌ Device returned Modbus exception!\n";
    logMsg += "Exception code: 0x" + String(response[2], HEX) + "\n";
    if (response[2] == 0x01) logMsg += "Reason: Illegal Function\n";
    else if (response[2] == 0x02) logMsg += "Reason: Illegal Data Address\n";
    else if (response[2] == 0x03) logMsg += "Reason: Illegal Data Value\n";
    else if (response[2] == 0x04) logMsg += "Reason: Slave Device Failure\n";
    logMsg += "\nThis device doesn't support Function Code 43\n";
    logMsg += "========================================\n";
    Serial.print(logMsg);
    logToWeb(logMsg);
    return;
  }

  // Verify response
  if (response[0] != SLAVE_ID || response[1] != 0x2B || response[2] != 0x0E) {
    logMsg = "❌ Invalid response format\n";
    logMsg += "Expected: Slave=" + String(SLAVE_ID) + ", FC=0x2B, MEI=0x0E\n";
    logMsg += "Received: Slave=" + String(response[0]) + ", FC=0x" + String(response[1], HEX) + ", MEI=0x" + String(response[2], HEX) + "\n";
    logMsg += "========================================\n";
    Serial.print(logMsg);
    logToWeb(logMsg);
    return;
  }

  // Parse device identification objects
  logMsg = "✅ Device responded! Parsing identification...\n\n";
  Serial.print(logMsg);
  logToWeb(logMsg);

  uint8_t readDeviceIdCode = response[3];
  uint8_t conformityLevel = response[4];
  uint8_t moreFollows = response[5];
  uint8_t nextObjectId = response[6];
  uint8_t numberOfObjects = response[7];

  logMsg = "Conformity Level: ";
  if (conformityLevel == 0x01) logMsg += "Basic";
  else if (conformityLevel == 0x02) logMsg += "Regular";
  else if (conformityLevel == 0x03) logMsg += "Extended";
  else logMsg += "Unknown (" + String(conformityLevel) + ")";
  logMsg += "\n";
  logMsg += "Number of objects: " + String(numberOfObjects) + "\n";
  logMsg += "More data follows: " + String(moreFollows ? "Yes" : "No") + "\n\n";
  Serial.print(logMsg);
  logToWeb(logMsg);

  // Parse objects
  int offset = 8;
  for (int i = 0; i < numberOfObjects && offset < bytesRead - 2; i++) {
    uint8_t objectId = response[offset++];
    uint8_t objectLength = response[offset++];

    String objectName = "";
    switch (objectId) {
      case 0x00: objectName = "VendorName"; break;
      case 0x01: objectName = "ProductCode"; break;
      case 0x02: objectName = "MajorMinorRevision"; break;
      case 0x03: objectName = "VendorUrl"; break;
      case 0x04: objectName = "ProductName"; break;
      case 0x05: objectName = "ModelName"; break;
      case 0x06: objectName = "UserApplicationName"; break;
      default: objectName = "Unknown_0x" + String(objectId, HEX); break;
    }

    String objectValue = "";
    for (int j = 0; j < objectLength && offset < bytesRead - 2; j++) {
      objectValue += (char)response[offset++];
    }

    logMsg = "📋 " + objectName + ": " + objectValue + "\n";
    Serial.print(logMsg);
    logToWeb(logMsg);
  }

  logMsg = "\n========================================\n";
  Serial.print(logMsg);
  logToWeb(logMsg);
}

void handleWebServer() {
  WiFiClient client = server.available();

  if (client) {
    String currentLine = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        if (c == '\n') {
          // End of HTTP request
          if (currentLine.length() == 0) {
            // Send HTTP response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println("Refresh: 5");  // Auto-refresh every 5 seconds
            client.println();

            // HTML page
            client.println("<!DOCTYPE html><html><head>");
            client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
            client.println("<style>");
            client.println("body{font-family:monospace;background:#1e1e1e;color:#d4d4d4;padding:20px;margin:0}");
            client.println("h1{color:#4ec9b0;border-bottom:2px solid #4ec9b0;padding-bottom:10px}");
            client.println(".info{background:#252526;padding:10px;border-left:4px solid #007acc;margin:10px 0}");
            client.println(".log{background:#252526;padding:5px;margin:3px 0;border-left:2px solid #608b4e}");
            client.println(".status{color:#4ec9b0;font-weight:bold}");
            client.println("</style>");
            client.println("</head><body>");

            client.println("<h1>Y&H HSI 5000U Monitor</h1>");

            client.println("<div class='info'>");
            client.println("<span class='status'>Status:</span> Online<br>");
            client.print("<span class='status'>Uptime:</span> ");
            client.print(millis() / 1000);
            client.println(" seconds<br>");
            client.print("<span class='status'>IP:</span> ");
            client.print(WiFi.localIP());
            client.println("<br>");
            client.print("<span class='status'>Total Logs:</span> ");
            client.print(logCount);
            client.println("</div>");

            client.println("<h2>Recent Logs (Last " + String(min(logCount, MAX_LOG_ENTRIES)) + ")</h2>");

            // Display logs in reverse order (newest first)
            int startIdx = (logIndex - 1 + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
            for (int i = 0; i < logCount; i++) {
              int idx = (startIdx - i + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
              client.println("<div class='log'>");
              client.println(logBuffer[idx]);
              client.println("</div>");
            }

            client.println("</body></html>");
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }

    client.stop();
  }
}

void loop() {
  ArduinoCloud.update();

  // Set deviceIP once when cloud is connected
  if (!deviceIPSet && ArduinoCloud.connected() && WiFi.status() == WL_CONNECTED) {
    deviceIP = "http://" + WiFi.localIP().toString();
    deviceIPSet = true;
    Serial.print("✅ Device IP set in cloud: ");
    Serial.println(deviceIP);
    logToWeb("Device IP published to cloud: " + deviceIP);
  }

  // Handle web server requests
  handleWebServer();

  // Your code here

  float cur = (vcc / 2 - vcc * avg()) / sens;

  keelPumpAmps = cur;

  
  
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
    logToWeb("\n⚠️  Scan already in progress!");
    return;
  }

  String msg = "\n========================================\n📸 STARTING PARAMETER SNAPSHOT\n========================================\nStep 1: Scanning Y&H registers (~90 sec)\nStep 2: Re-scan to detect telemetry (~15 sec)\nNormal polling continues during scan.\n========================================\n";
  Serial.print(msg);
  logToWeb(msg);

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
    logToWeb("\n⚠️  Scan already in progress!");
    return;
  }

  if (!snapshotTaken) {
    String msg = "\n❌ ERROR: No snapshot taken yet!\nTake a snapshot first by sending 'T'\n";
    Serial.print(msg);
    logToWeb(msg);
    return;
  }

  String msg = "\n========================================\n🔍 COMPARING PARAMETERS TO SNAPSHOT\n========================================\nRe-reading all snapshot registers...\nThis will take ~15 seconds.\n========================================\n";
  Serial.print(msg);
  logToWeb(msg);

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
      String msg = "\n========================================\n✅ Step 1 complete: " + String(snapshotCount) + " registers found\n========================================\nStep 2: Detecting telemetry registers...\n(waiting 5 seconds for values to change)\n========================================\n";
      Serial.print(msg);
      logToWeb(msg);

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

      // Start new buffer for this range
      String rangeStartStr = String(range->start, HEX);
      while (rangeStartStr.length() < 4) rangeStartStr = "0" + rangeStartStr;
      rangeStartStr.toUpperCase();
      String rangeEndStr = String(endAddr, HEX);
      while (rangeEndStr.length() < 4) rangeEndStr = "0" + rangeEndStr;
      rangeEndStr.toUpperCase();
      registerLogBuffer = "Range 0x" + rangeStartStr + "-0x" + rangeEndStr + ":\n";
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

      // Format address with leading zeros
      String addrStr = String(addr, HEX);
      while (addrStr.length() < 4) addrStr = "0" + addrStr;
      addrStr.toUpperCase();

      String msg = "  ✓ 0x" + addrStr + " = " + String(value);
      Serial.println(msg);

      // Add to buffer instead of sending immediately
      registerLogBuffer += msg + "\n";
    }

    // Move to next register
    currentRegisterIndex++;
    if (currentRegisterIndex >= range->count) {
      // Done with this range, check if we found any registers
      bool foundAny = false;
      for (int i = 0; i < snapshotCount; i++) {
        if (paramSnapshot[i].address >= range->start &&
            paramSnapshot[i].address < range->start + range->count) {
          foundAny = true;
          break;
        }
      }

      if (!foundAny) {
        Serial.println("none");
        registerLogBuffer += "  (none)\n";
      }

      // Send accumulated buffer for this range to webhook
      if (registerLogBuffer.length() > 0) {
        logToWeb(registerLogBuffer);
        registerLogBuffer = "";  // Clear buffer for next range
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

      String msg = "\n========================================\n✅ Snapshot complete!\n   Total registers: " + String(snapshotCount) + "\n   Stable (settings): " + String(stableCount) + "\n   Volatile (telemetry): " + String(volatileCount) + " [ignored]\n========================================\nNOW: Change a setting on your inverter\nTHEN: Send 'C' to compare\n========================================\n";
      Serial.print(msg);
      logToWeb(msg);
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

      String msg = "\n========================================\n✅ Comparison complete!\n========================================\n";
      Serial.print(msg);
      logToWeb(msg);
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
      // Format address with leading zeros
      String addrStr = String(addr, HEX);
      while (addrStr.length() < 4) addrStr = "0" + addrStr;
      addrStr.toUpperCase();

      String msg = "🎯 SETTING CHANGED!\n   Address: 0x" + addrStr + " (" + String(addr) + ")\n   Old: " + String(oldValue) + " → New: " + String(newValue) + "\n   ⭐ WRITABLE SETTING REGISTER!\n";
      Serial.print(msg);
      logToWeb(msg);
    }
    
    currentRangeIndex++;
  }
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
  













