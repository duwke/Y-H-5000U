# Y&H HSI 5000U / SRNE Solar Inverter Monitor for Arduino R4

Based on SRNE Modbus RTU protocol

## Hardware

- RS485 to TTL converter module (auto-direction type with TX/RX only)

## Connections

### Y&H RJ45 to RS485 Module
- Y&H RJ45 Pin 7 (RS485-A) → RS485 Module A
- Y&H RJ45 Pin 8 (RS485-B) → RS485 Module B
- Y&H RJ45 Pin 1 (GND) → RS485 Module GND & Arduino GND

### RS485 Module to Arduino
- RS485 Module VCC → Arduino 5V
- RS485 Module RX → Arduino Serial1 TX (Pin 1)
- RS485 Module TX → Arduino Serial1 RX (Pin 0)

## Arduino IoT Cloud Variables Required

| Variable | Type | Access | Callback |
|----------|------|--------|----------|
| `batteryCurrent` | CloudElectricCurrent | Read | - |
| `pvCurrent` | CloudElectricCurrent | Read | - |
| `batteryVoltage` | CloudElectricPotential | Read | - |
| `pvVoltage` | CloudElectricPotential | Read | - |
| `batterySOC` | int | Read | - |
| `loadPower` | CloudPower | Read | - |
| `pvPower` | CloudPower | Read | - |
| `enableBuzzer` | bool | Read & Write | `onEnableBuzzerChange` |
| `takeSnapshot` | bool | Read & Write | `onTakeSnapshotChange` |
| `compareSnapshot` | bool | Read & Write | `onCompareSnapshotChange` |

## Control

Control everything via Arduino IoT Cloud dashboard!
