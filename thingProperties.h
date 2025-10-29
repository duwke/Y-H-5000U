#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "arduino_secrets.h"

void onCompareSnapshotChange();
void onTakeSnapshotChange();

// Cloud Variables
CloudElectricCurrent batteryCurrent;
CloudElectricCurrent pvCurrent;
CloudElectricPotential batteryVoltage;
CloudElectricPotential pvVoltage;
float keelPumpAmps;
CloudPower loadPower;
CloudPower pvPower;
bool compareSnapshot;
bool takeSnapshot;
String deviceIP;

// WiFi Connection Handler
WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_WIFI_SSID, SECRET_WIFI_PASS);

void initProperties(){
  ArduinoCloud.addProperty(batteryCurrent, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pvCurrent, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(batteryVoltage, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pvVoltage, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(keelPumpAmps, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(loadPower, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pvPower, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(compareSnapshot, READWRITE, ON_CHANGE, onCompareSnapshotChange);
  ArduinoCloud.addProperty(takeSnapshot, READWRITE, ON_CHANGE, onTakeSnapshotChange);
  ArduinoCloud.addProperty(deviceIP, READ, ON_CHANGE, NULL);
}
