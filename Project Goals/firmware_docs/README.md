# Heltec LoRa Kill Switch System

This folder contains the Arduino/ESP32 code for the Heltec LoRa-based hardware kill switch system used on the Cube Orange outdoor rover.

## System Overview

The kill switch system uses two Heltec WiFi LoRa 32 V3 modules:
- **Transmitter** (handheld): User carries this and can enable/disable the rover
- **Receiver** (on rover): Connected to Jetson via USB, controls relay for motor power

## Files in this Folder

### Relay-Based Kill Switch (Hardware Fail-Safe)
These files implement **direct relay control** via LoRa commands:

| File | Purpose |
|------|---------|
| `LoRa_KillSwitch_Transmitter.ino` | Handheld transmitter with toggle switch |
| `LoRa_KillSwitch_Receiver_Relay.ino` | Receiver with **RELAY_PIN control** (GPIO 21) |

**How it works:**
1. Transmitter reads toggle switch position
2. Sends "ENABLE" or "DISABLE" over LoRa
3. Receiver sets `RELAY_PIN` HIGH or LOW to control motor power relay

**Key Code:**
```cpp
#define RELAY_PIN   21  // Controls power relay

// In loop():
if (message == "ENABLE") {
    digitalWrite(RELAY_PIN, HIGH);  // Motor power ON
} else if (message == "DISABLE") {
    digitalWrite(RELAY_PIN, LOW);   // Motor power OFF
}
```

### Software Mode Kill Switch (Enhanced with Battery Monitoring)
These files send software commands to Jetson with battery status forwarding:

| File | Purpose |
|------|---------|
| `Transmitter_with_Battery_Display.ino` | Handheld with OLED showing battery status |
| `Receiver_with_Battery_Monitor.ino` | Sends "AUTONOMOUS"/"STOP" to Jetson serial |

**How it works:**
1. Transmitter sends "AUTONOMOUS" or "STOP" commands
2. Receiver forwards to Jetson via Serial: `MODE:AUTONOMOUS` or `MODE:STOP`
3. Receiver receives battery alerts from Jetson and forwards to transmitter
4. No direct relay control - Jetson handles the command

## Choosing the Right Version

### Use Relay Version (`LoRa_KillSwitch_Receiver_Relay.ino`) if:
- You want a **hardware fail-safe** that works even if Jetson crashes
- The relay directly cuts motor controller power
- Simple on/off control is sufficient

### Use Software Version (`Receiver_with_Battery_Monitor.ino`) if:
- You want battery status displayed on handheld
- Jetson needs to know the mode for autonomous behavior
- You want more sophisticated state management

## Recommended Setup for Cube Orange Rover

For the Jetson Cube Orange Outdoor Rover, we recommend a **hybrid approach**:

1. **Use the Relay Version** as the primary safety system
2. **Optionally add Jetson heartbeat** where:
   - Jetson sends periodic heartbeat to Heltec
   - If heartbeat stops (Jetson crash), Heltec activates relay to cut power
   - This provides both software control AND hardware fail-safe

## Hardware Configuration

### Relay Wiring (for relay-based version)
```
Heltec GPIO 21 --> Relay Module Signal
Relay Common  --> Motor Controller Power (+)
Relay NO      --> Battery (+)
```

When relay is energized (ENABLE): Motor controller powered
When relay is de-energized (DISABLE): Motor controller unpowered

### Serial Connection (for software version)
```
Heltec USB --> Jetson USB Hub --> /dev/ttyUSB2 (typical)
```

### LoRa Parameters (must match on both units)
- Frequency: 915.0 MHz (US) / 868.0 MHz (EU)
- Spreading Factor: 10
- Bandwidth: 125.0 kHz
- Coding Rate: 8
- Output Power: 22 dBm (max)
- Sync Word: 0x12 (change for multiple rovers)

## Channel Separation

If you have multiple rovers, change the `SYNC_WORD` in both transmitter and receiver:
- Rover 1: `0x12`
- Rover 2: `0x34`
- Rover 3: `0x56`

## Required Libraries

Install via Arduino Library Manager:
- **RadioLib** - SX1262 LoRa support
- **ArduinoJson** - JSON parsing (battery monitor version only)
- **Heltec ESP32 Dev-Boards** - Board support and OLED display

## Flashing Instructions

1. Open Arduino IDE
2. Select **Tools > Board > Heltec WiFi LoRa 32(V3)**
3. Select correct COM port
4. Upload the appropriate `.ino` file

## Testing

### Test Relay Version
1. Upload `LoRa_KillSwitch_Receiver_Relay.ino` to receiver
2. Upload `LoRa_KillSwitch_Transmitter.ino` to transmitter
3. Power on both units
4. Toggle switch on transmitter should control relay on receiver
5. Measure voltage on GPIO 21 with multimeter (3.3V = enabled, 0V = disabled)

### Test Software Version
1. Upload receiver code and connect to Jetson
2. Open serial monitor at 115200 baud
3. Send command from transmitter
4. Verify `MODE:AUTONOMOUS` or `MODE:STOP` appears on serial

## Safety Notes

- **Default is OFF/DISABLED**: Rover always starts with motors off
- **Test before field deployment**: Verify response at close range first
- **Keep transmitter charged**: Dead transmitter = no control
- **Physical override**: Always have a manual disconnect accessible

## Battery Monitoring (Software Version Only)

The software version supports battery status display. See communication flow:

```
Jetson (battery_monitor.py)
    | Serial JSON
    v
Receiver (Heltec on rover)
    | LoRa JSON
    v
Transmitter (Handheld)
    --> Display battery status
```

Battery Alert Format (from Jetson):
```json
{
  "type": "BATTERY_ALERT",
  "voltage": 11.5,
  "state": "GOOD",
  "percent": 75
}
```

---

**Last Updated:** November 23, 2025
**Board:** Heltec WiFi LoRa 32 V3 (HT-WB32LAF)
**Source:** Consolidated from archive/Arduino Code/Heltec Code/ and Jetson Simple Rover project
