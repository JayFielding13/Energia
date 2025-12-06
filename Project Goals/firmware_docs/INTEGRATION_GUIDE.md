# Heltec Gen 2 - Integration and Testing Guide

**Status:** Ready for testing
**Date:** November 25, 2025

## Overview

This guide covers uploading, testing, and integrating the Gen 2 Heltec safety switch system with the Jetson Cube Orange rover.

## Files Created

### Arduino Code (Heltec Modules)
- **Transmitter:** `Heltec Gen 2/Transmitter/LoRa_KillSwitch_Transmitter_Gen2.ino`
- **Receiver:** `Heltec Gen 2/Receiver/LoRa_KillSwitch_Receiver_Gen2.ino`

### Python Service (Jetson)
- **Heartbeat Service:** `rover/scripts/heltec_heartbeat_service.py`

## Phase 1: Upload Arduino Code

### Step 1.1: Install Required Libraries

Open Arduino IDE and install via Library Manager:

1. **RadioLib** (by Jan Gromeš)
   - Tools → Manage Libraries → Search "RadioLib" → Install

2. **Heltec ESP32 Dev-Boards** (Board Support)
   - File → Preferences → Additional Board Manager URLs:
   ```
   https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.7/package_heltec_esp32_index.json
   ```
   - Tools → Board → Boards Manager → Search "Heltec" → Install

### Step 1.2: Upload Transmitter Code

1. Open `LoRa_KillSwitch_Transmitter_Gen2.ino` in Arduino IDE
2. Select board: **Tools → Board → Heltec WiFi LoRa 32(V3)**
3. Select correct COM port: **Tools → Port → COMx** (Windows) or **/dev/ttyUSBx** (Linux)
4. Upload: **Sketch → Upload** or press Ctrl+U
5. Open Serial Monitor (115200 baud) to verify:
   ```
   LoRa Kill Switch - Transmitter Gen 2
   Sync Word (Channel): 0x12
   Initializing LoRa... success!
   ```

### Step 1.3: Upload Receiver Code

1. Open `LoRa_KillSwitch_Receiver_Gen2.ino` in Arduino IDE
2. Same board selection as transmitter
3. Select receiver's COM port
4. Upload code
5. Verify in Serial Monitor:
   ```
   LoRa Kill Switch - Receiver Gen 2
   Two-Layer Safety System
   Sync Word (Channel): 0x12
   [SAFETY] Relay initialized to OFF (safe mode)
   ```

## Phase 2: Bench Testing (No Rover Connection)

### Test 2.1: Basic LoRa Communication

**Setup:**
- Both Heltec modules powered (USB)
- Toggle switch on transmitter in OFF position
- Both serial monitors open

**Test Steps:**
1. Toggle switch OFF → Verify receiver shows "STOP"
2. Toggle switch ON → Verify receiver shows "AUTONOMOUS"
3. Check OLED displays show correct mode on both units
4. Verify message counter increments on transmitter
5. Check RSSI values on receiver display (should be strong, e.g., -30 to -50 dBm)

**Expected Results:**
- ✓ Commands received within 1 second
- ✓ Relay clicks when toggling (if audible)
- ✓ OLED shows matching state on both units
- ✓ Serial shows "Message sent successfully!"

### Test 2.2: LoRa Heartbeat

**Setup:**
- Toggle switch to ON (AUTONOMOUS)
- Watch receiver serial monitor

**Test Steps:**
1. Enable AUTONOMOUS mode (toggle ON)
2. Observe transmitter serial output
3. Count messages - should send every 2 seconds

**Expected Results:**
- ✓ "[Heartbeat] Sending periodic AUTONOMOUS" every 2 seconds
- ✓ Receiver shows increasing message count
- ✓ No "SEND FAILED" errors

### Test 2.3: LoRa Timeout Test

**Setup:**
- Start with AUTONOMOUS mode enabled
- Receiver relay should be ON

**Test Steps:**
1. Enable AUTONOMOUS (toggle ON)
2. Wait 3 seconds for relay to engage
3. Unplug transmitter USB power
4. Watch receiver serial monitor closely
5. Count seconds until emergency shutdown

**Expected Results:**
- ✓ Receiver detects timeout after ~5 seconds
- ✓ Serial shows: "!!! SAFETY VIOLATION: LoRa connection lost !!!"
- ✓ Serial shows: "!!! EMERGENCY SHUTDOWN: LORA_TIMEOUT !!!"
- ✓ Relay immediately disengages (click sound)
- ✓ OLED shows "EMERGENCY!" and "LORA_TIMEOUT"
- ✓ Display flashes 3 times

**CRITICAL:** If timeout doesn't trigger, DO NOT proceed to rover testing!

## Phase 3: Jetson Integration

### Step 3.1: Install Python Dependencies on Jetson

```bash
# SSH into Jetson
ssh jay@192.168.8.110  # or your Jetson's IP

# Install pyserial
pip3 install pyserial

# Or system-wide:
sudo apt update
sudo apt install python3-serial
```

### Step 3.2: Copy Heartbeat Service to Jetson

```bash
# From your development machine
scp rover/scripts/heltec_heartbeat_service.py jay@192.168.8.110:~/rover/

# Or if Jetson is accessible via file share
# Copy manually to /home/jay/rover/ on Jetson
```

### Step 3.3: Test Heartbeat Service

**Setup:**
- Receiver connected to Jetson via USB
- Transmitter in STOP mode (toggle OFF)

**Test Steps:**

1. **Identify Heltec port:**
   ```bash
   # On Jetson
   ls -la /dev/ttyUSB*
   # or
   dmesg | grep tty
   ```

2. **Test heartbeat script:**
   ```bash
   cd ~/rover
   python3 scripts/heltec_heartbeat_service.py
   ```

3. **Verify output:**
   ```
   ============================================================
   Heltec Heartbeat Service - Jetson Cube Orange Rover
   ============================================================
   [INFO] Found Heltec on port: /dev/ttyUSB0
   [INFO] Connected to Heltec on /dev/ttyUSB0 @ 115200 baud
   [INFO] Heartbeat service started
          Sending heartbeat every 3.0 seconds
   [HEARTBEAT] Sent 10 heartbeats. Battery: 12.4V, CPU: 45.2°C
   ```

4. **Check receiver serial monitor:**
   - Should show: `[JETSON] Heartbeat received`
   - Should update every 3 seconds

**Expected Results:**
- ✓ Service auto-detects Heltec port
- ✓ Heartbeats send every 3 seconds
- ✓ Receiver acknowledges heartbeats
- ✓ Receiver OLED shows "Jetson: 0-3s" (updating)

### Test 3.4: Jetson Timeout Test

**Setup:**
- Heartbeat service running
- Toggle switch in AUTONOMOUS (ON)
- Receiver relay should be ON

**Test Steps:**
1. Enable AUTONOMOUS mode (toggle ON)
2. Verify relay engages
3. Stop heartbeat service: Press Ctrl+C
4. Watch receiver serial closely
5. Count seconds until emergency shutdown

**Expected Results:**
- ✓ Receiver detects Jetson timeout after ~6 seconds
- ✓ Serial shows: "!!! SAFETY VIOLATION: Jetson not responding !!!"
- ✓ Serial shows: "!!! EMERGENCY SHUTDOWN: JETSON_TIMEOUT !!!"
- ✓ Relay disengages immediately
- ✓ OLED shows "EMERGENCY!" and "JETSON_TIMEOUT"

**CRITICAL:** This test validates Layer 2 safety. If it fails, investigate heartbeat timing.

### Test 3.5: Combined Safety Test

**Test both layers fail independently:**

1. **Test LoRa timeout with Jetson healthy:**
   - Start heartbeat service (Jetson healthy)
   - Enable AUTONOMOUS
   - Unplug transmitter
   - Should shutdown with "LORA_TIMEOUT"

2. **Test Jetson timeout with LoRa healthy:**
   - Keep transmitter powered and enabled
   - Kill heartbeat service on Jetson
   - Should shutdown with "JETSON_TIMEOUT"

3. **Verify display shows correct failure reason**

## Phase 4: Rover Integration

### Step 4.1: Install Receiver in Rover

**Hardware Setup:**
1. Mount receiver Heltec near Jetson (for short USB cable)
2. Connect receiver USB to Jetson USB hub
3. Wire relay output (GPIO 21) to motor controller relay:
   ```
   Heltec GPIO 21 (3.3V logic) → Relay Module Signal Input
   Relay Common (COM) → Motor Controller Power Input
   Relay NO (Normally Open) → Battery Positive
   ```
4. Test relay manually:
   - Measure voltage at GPIO 21: 0V = OFF, 3.3V = ON
   - Verify relay clicks when toggling switch

### Step 4.2: Configure Heartbeat Service Autostart

Create systemd service for automatic startup:

```bash
# On Jetson, create service file
sudo nano /etc/systemd/system/heltec-heartbeat.service
```

**Service file content:**
```ini
[Unit]
Description=Heltec Safety Switch Heartbeat Service
After=network.target

[Service]
Type=simple
User=jay
WorkingDirectory=/home/jay/rover
ExecStart=/usr/bin/python3 /home/jay/rover/scripts/heltec_heartbeat_service.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

**Enable and start service:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable heltec-heartbeat.service
sudo systemctl start heltec-heartbeat.service

# Check status
sudo systemctl status heltec-heartbeat.service

# View logs
journalctl -u heltec-heartbeat.service -f
```

### Step 4.3: Integrate with Rover Control Code

The heartbeat service outputs MODE commands that your rover software can listen to:

**Serial Output Format:**
```
MODE:STOP
MODE:AUTONOMOUS
```

**JSON Format:**
```json
{"mode":"AUTONOMOUS","rssi":-45,"relay":true,"time":123456}
```

**Integration Options:**

1. **Shared Memory/File:** Heartbeat service writes mode to file, rover reads it
2. **ROS2 Topic:** Modify heartbeat service to publish to ROS topic
3. **Direct Serial Parsing:** Rover code reads from same serial port (not recommended)

**Recommended: Add ROS2 publishing to heartbeat service**

Modify `heltec_heartbeat_service.py` to publish mode changes:

```python
# Add to imports
import rclpy
from std_msgs.msg import String

# In HeltecHeartbeatService class, add publisher:
self.mode_publisher = self.create_publisher(String, '/rover/safety_mode', 10)

# In read_commands(), publish mode changes:
if mode != self.last_mode:
    msg = String()
    msg.data = mode
    self.mode_publisher.publish(msg)
```

## Phase 5: Field Testing

### Test 5.1: Range Testing

**Setup:**
- Rover with receiver installed and powered
- Transmitter with you
- Open area (parking lot, field)

**Test Steps:**
1. Start at 10 meters - verify AUTONOMOUS works
2. Walk to 50 meters - check RSSI (should still be good)
3. Walk to 100 meters - verify commands still work
4. Walk beyond range until timeout triggers
5. Return to range - re-enable should work

**Record RSSI at each distance:**
- 10m: _____ dBm
- 50m: _____ dBm
- 100m: _____ dBm
- Timeout at: _____ meters

**Expected Results:**
- ✓ Good communication to at least 100m
- ✓ Timeout triggers reliably when out of range
- ✓ Recovery when returning to range

### Test 5.2: Battery Brown-out Simulation

**WARNING: This test requires careful monitoring**

**Setup:**
- Rover on bench with motors disabled (unplug motor power)
- Battery voltage monitoring visible
- Controlled power supply if possible

**Test Steps:**
1. Start with full battery
2. Enable AUTONOMOUS mode
3. Load system to drain battery (run compute tasks)
4. Monitor Jetson for shutdown
5. Verify Heltec triggers JETSON_TIMEOUT when Jetson dies

**CAUTION:** Don't damage Jetson with too-low voltage. Have external power ready.

### Test 5.3: Full System Test

**Final validation:**

1. **Startup sequence:**
   - Power on rover (Jetson boots)
   - Heartbeat service auto-starts
   - Receiver shows "Jetson: Wait..." → "Jetson: 0s"
   - Toggle switch OFF (STOP mode)

2. **Enable sequence:**
   - Toggle switch ON
   - Verify relay engages
   - Verify OLED shows "AUTONOMOUS"
   - Test motors respond

3. **Disable sequence:**
   - Toggle switch OFF
   - Verify immediate relay disengagement
   - Verify motors stop

4. **Emergency scenarios:**
   - Transmitter out of range → 5 sec timeout
   - Jetson crash (kill heartbeat) → 6 sec timeout
   - Both recoverable after fixing issue

## Troubleshooting

### Issue: Relay doesn't toggle

**Checks:**
- Measure voltage at GPIO 21 (should be 3.3V when ON)
- Verify relay module is 3.3V compatible (not 5V only)
- Check relay wiring: Signal, VCC, GND all connected
- Test relay with external 3.3V source

### Issue: LoRa messages not received

**Checks:**
- Verify SYNC_WORD matches on both (0x12)
- Check frequency is 915.0 MHz on both
- Verify antennas connected properly
- Check RSSI value (very low = antenna issue)
- Try reducing distance to 1 meter

### Issue: Jetson heartbeat not detected

**Checks:**
- Run `ls /dev/ttyUSB*` - is Heltec detected?
- Try manual port: `python3 heltec_heartbeat_service.py /dev/ttyUSB0`
- Check baud rate matches (115200)
- Verify USB cable supports data (not just power)
- Check receiver serial monitor for "HEARTBEAT" messages

### Issue: Timeouts trigger too quickly/slowly

**Adjust timeout values in receiver code:**

```cpp
#define LORA_TIMEOUT_MS 5000      // Increase for slower timeout
#define JETSON_TIMEOUT_MS 6000    // Increase for slower timeout
```

Then adjust transmitter heartbeat:

```cpp
#define HEARTBEAT_INTERVAL 2000   // Decrease for faster heartbeat
```

**Rule: Timeout should be 2-3x heartbeat interval**

## Safety Checklist Before Field Operation

- [ ] Both Arduino codes uploaded and tested
- [ ] LoRa timeout test passed (5 sec shutdown)
- [ ] Jetson timeout test passed (6 sec shutdown)
- [ ] Relay wiring tested and verified
- [ ] Heartbeat service runs on Jetson boot
- [ ] Range tested to at least 50 meters
- [ ] Emergency shutdown tested and works
- [ ] Display shows correct status
- [ ] Manual toggle switch works in all scenarios
- [ ] Battery monitoring functional
- [ ] Physical emergency stop accessible

## Next Steps After Testing

1. **Document test results** in this file
2. **Update README.md** with test outcomes
3. **Create backup** of working configuration
4. **Consider Gen 3 features:**
   - Battery voltage display on transmitter OLED
   - Two-way acknowledgment
   - GPS position logging
   - Signal strength warnings

---

**Testing Status:** [ ] Not started [ ] In progress [ ] Complete

**Last Test Date:** _________________

**Tested By:** _________________

**Notes:**
