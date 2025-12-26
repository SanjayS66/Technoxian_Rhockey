# ESP32 Robot Hockey Controller - Complete Code Explanation

## Table of Contents
1. [Overview](#overview)
2. [Hardware Components](#hardware-components)
3. [Pin Configuration](#pin-configuration)
4. [System Architecture](#system-architecture)
5. [Code Flow](#code-flow)
6. [Detailed Component Explanation](#detailed-component-explanation)
7. [Controller Input Processing](#controller-input-processing)
8. [PWM Signal Generation](#pwm-signal-generation)
9. [Differential Drive Logic](#differential-drive-logic)
10. [Configuration Parameters](#configuration-parameters)
11. [Troubleshooting](#troubleshooting)

---

## Overview

This ESP32-based robot hockey controller uses **Bluepad32** library to receive wireless input from Bluetooth game controllers (PS4/Xbox) and translates those inputs into **PWM (Pulse Width Modulation) signals** to control DC motors through motor drivers. The system implements **differential drive** mechanics for precise robot movement control.

### Key Features:
- Bluetooth controller support (PS4, Xbox)
- Differential drive control for smooth turning
- Dynamic speed control with acceleration/braking
- Point turn capability for in-place rotation
- Emergency stop functionality
- Customizable speed and sensitivity parameters

---

## Hardware Components

### Essential Components:
1. **ESP32 Development Board** - Main microcontroller
2. **Motor Driver** - SmartElex 15D or MDD10A
3. **DC Motors** - 12V or 24V (2 for 2-wheel, 4 for 4-wheel configuration)
4. **Game Controller** - PS4 DualShock or Xbox controller
5. **Power Supply** - 12V or 24V battery pack
6. **Buck Converter** - To power ESP32 from main battery

### Wiring:
- Motor drivers connected to ESP32 GPIO pins
- Motors connected to motor driver outputs
- Power supply to motors and ESP32 (through buck converter)
- USB cable for programming and serial monitoring

---

## Pin Configuration

### 2-Wheel Configuration:
```cpp
// Motor Control Pins
int rmdpwm = 33;   // Right motor PWM signal pin
int rmddir = 32;   // Right motor direction control pin
int lmdpwm = 26;   // Left motor PWM signal pin
int lmddir = 25;   // Left motor direction control pin
```

### 4-Wheel Configuration:
```cpp
// Right Side Motors
int rmdpwm1 = 17;    // Right motor 1 PWM
int rmdpwm2 = 4;     // Right motor 2 PWM
int rmddir1 = 5;     // Right motor 1 direction
int rmddir2 = 16;    // Right motor 2 direction

// Left Side Motors
int lmdpwm1 = 21;    // Left motor 1 PWM
int lmdpwm2 = 1;     // Left motor 2 PWM
int lmddir1 = 19;    // Left motor 1 direction
int lmddir2 = 3;     // Left motor 2 direction
```

**Pin Functions:**
- **PWM Pins**: Generate variable speed signals (0-255 duty cycle)
- **Direction Pins**: Control motor rotation direction (HIGH/LOW)

---

## System Architecture

```
┌─────────────────┐
│  Game Controller│
│  (PS4/Xbox)     │
└────────┬────────┘
         │ Bluetooth
         ▼
┌─────────────────┐
│     ESP32       │
│  ┌───────────┐  │
│  │ Bluepad32 │  │
│  │  Library  │  │
│  └─────┬─────┘  │
│        │        │
│  ┌─────▼─────┐  │
│  │Controller │  │
│  │Processing │  │
│  └─────┬─────┘  │
│        │        │
│  ┌─────▼─────┐  │
│  │Differential│  │
│  │   Drive   │  │
│  │  Mixer    │  │
│  └─────┬─────┘  │
│        │        │
│  ┌─────▼─────┐  │
│  │PWM Signal │  │
│  │Generation │  │
│  └─────┬─────┘  │
└────────┼────────┘
         │ GPIO Pins
         ▼
┌─────────────────┐
│  Motor Driver   │
│  (SmartElex/    │
│   MDD10A)       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│   DC Motors     │
└─────────────────┘
```

---

## Code Flow

### 1. Setup Phase (Runs Once)
```cpp
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Configure PWM channels
    ledcSetup(channelL, freq, res);  // Left motor PWM channel
    ledcSetup(channelR, freq, res);  // Right motor PWM channel
    
    // Attach PWM channels to GPIO pins
    ledcAttachPin(lmdpwm, channelL);
    ledcAttachPin(rmdpwm, channelR);
    
    // Set direction pins as outputs
    pinMode(lmddir, OUTPUT);
    pinMode(rmddir, OUTPUT);
    
    // Initialize motors to stopped state
    setMotorSpeeds(0, 0);
    
    // Setup Bluepad32 controller callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
}
```

### 2. Main Loop (Runs Continuously)
```cpp
void loop() {
    // Check for controller data updates
    if (BP32.update()) {
        processControllers();  // Process controller inputs
    }
    vTaskDelay(1);  // Small delay to prevent watchdog timeout
}
```

### 3. Controller Processing Pipeline
```
Controller Input → Read Joystick/Button Values → Apply Deadzone → 
Normalize Values → Calculate Speed → Apply Turn Sensitivity → 
Differential Drive Mixing → Constrain Values → Send PWM Signals → 
Control Motors
```

---

## Detailed Component Explanation

### 1. Bluepad32 Library Integration

**What is Bluepad32?**
- Arduino library that enables Bluetooth controller support on ESP32
- Handles pairing, connection, and data reception from game controllers
- Supports multiple controller types (PS4, Xbox, Switch Pro, etc.)

**Controller Connection Callback:**
```cpp
void onConnectedController(ControllerPtr ctl) {
    // Called automatically when a controller connects
    // Stores controller pointer for future access
    myControllers[i] = ctl;
}
```

**Controller Disconnection Callback:**
```cpp
void onDisconnectedController(ControllerPtr ctl) {
    // Called when controller disconnects
    // Stops motors for safety
    setMotorSpeeds(0, 0);
    myControllers[i] = nullptr;
}
```

---

### 2. Reading Controller Inputs

The controller provides multiple input types:

#### Analog Inputs (Joysticks):
```cpp
float yAxis = (float)(ctl->axisY());   // Left joystick Y: -511 to +512
float xAxis = (float)(ctl->axisRX());  // Right joystick X: -511 to +512
```

- **yAxis**: Controls forward/backward movement
- **xAxis**: Controls left/right turning

#### Analog Triggers:
```cpp
int accel = ctl->throttle();  // Right trigger: 0-1023
int brake = ctl->brake();     // Left trigger: 0-1023
```

- **throttle**: Increases speed beyond base speed
- **brake**: Decreases speed or stops

#### Digital Buttons:
```cpp
bool ccwpturn = ctl->buttons() & 0x0004; // X button (Xbox) / Square (PS4)
bool cwpturn = ctl->buttons() & 0x0002;  // B button (Xbox) / Circle (PS4)
bool emergencyStop = (ctl->buttons() & 0x0030) == 0x0030; // L1 + R1
```

---

### 3. Input Processing and Deadzone

**Why Deadzone?**
Controller joysticks may not return exactly 0 when centered. Deadzone eliminates unintentional movement.

```cpp
int joystickDeadzone = 70;  // Ignore values below this threshold

// Apply deadzone
if (abs(yAxis) < joystickDeadzone) yAxis = 0;
if (abs(xAxis) < joystickDeadzone) xAxis = 0;
```

**Normalization:**
```cpp
// Convert from -511...+512 to -1.0...+1.0
yAxis = yAxis / 512.0;
xAxis = xAxis / 512.0;

// Ensure values stay within bounds
yAxis = constrain(yAxis, -1.0, 1.0);
xAxis = constrain(xAxis, -1.0, 1.0);
```

---

### 4. Speed Control System

**Three Speed Modes:**

1. **Base Speed** (Default cruising):
```cpp
int baseSpeed = 90;  // Default speed when no throttle/brake applied
```

2. **Maximum Speed** (Full throttle):
```cpp
int maxSpeed = 255;  // Maximum PWM value (full power)
```

3. **Dynamic Speed** (Variable):
```cpp
int currentSpeed = baseSpeed;  // Tracks current target speed
```

**Speed Ramping** (Prevents jerky acceleration):
```cpp
int speedChangeRate = 6;  // Speed changes gradually

// Gradually increase speed
currentSpeed = min(currentSpeed + speedChangeRate, targetSpeed);

// Gradually decrease speed
currentSpeed = max(currentSpeed - speedChangeRate, targetSpeed);
```

**Throttle/Brake Processing:**
```cpp
if (accel > 50) {
    // Map throttle (0-1023) to speed range (baseSpeed-maxSpeed)
    int targetSpeed = map(accel, 0, 1023, baseSpeed, maxSpeed);
    currentSpeed = min(currentSpeed + speedChangeRate, targetSpeed);
}
else if (brake > 50) {
    // Calculate brake reduction
    int brakeAmount = map(brake, 0, 1023, 0, baseSpeed);
    int targetSpeed = max(baseSpeed - brakeAmount, 0);
    currentSpeed = max(currentSpeed - speedChangeRate, targetSpeed);
}
```

---

### 5. Turn Sensitivity

**Problem:** Full turning at high speeds can be difficult to control.

**Solution:** Reduce turn input sensitivity:
```cpp
float turnSensitivity = 0.55;  // 0.0 to 1.0 (lower = wider turns)
xAxis *= turnSensitivity;
```

**Effect:**
- `turnSensitivity = 1.0`: Full turn response (tight turns)
- `turnSensitivity = 0.5`: Half turn response (wider turns)
- `turnSensitivity = 0.0`: No turning (straight only)

---

### 6. Differential Drive Logic

**What is Differential Drive?**
Differential drive robots control direction by varying the speed of left and right motors.

**Mathematical Mixing:**
```cpp
// Scale values by current speed
float scale = (float)currentSpeed / (float)maxSpeed;
int forward = (int)(yAxis * scale * maxSpeed);  // Forward/backward component
int turn = (int)(xAxis * scale * maxSpeed);     // Turn component

// Mix forward and turn to get individual motor speeds
int leftSpeed  = forward - turn;   // Left motor
int rightSpeed = forward + turn;   // Right motor
```

**Movement Examples:**

| Forward | Turn | Left Motor | Right Motor | Result |
|---------|------|------------|-------------|--------|
| 100     | 0    | 100        | 100         | Straight forward |
| 100     | 50   | 50         | 150         | Forward + right turn |
| 100     | -50  | 150        | 50          | Forward + left turn |
| 0       | 100  | -100       | 100         | Spin right |
| -100    | 0    | -100       | -100        | Straight backward |

**Constraining Values:**
```cpp
leftSpeed  = constrain(leftSpeed, -max_pwm, max_pwm);
rightSpeed = constrain(rightSpeed, -max_pwm, max_pwm);
```
Ensures motor speeds stay within valid PWM range (-255 to +255).

---

### 7. Point Turn Feature

**Point Turn** = Robot rotates in place without moving forward/backward.

**Counter-Clockwise (CCW) Point Turn:**
```cpp
if (ccwpturn) {  // X button (Xbox) or Square (PS4)
    setMotorSpeeds(currentSpeed, -currentSpeed);
    // Left forward + Right backward = CCW rotation
}
```

**Clockwise (CW) Point Turn:**
```cpp
if (cwpturn) {  // B button (Xbox) or Circle (PS4)
    setMotorSpeeds(-currentSpeed, currentSpeed);
    // Left backward + Right forward = CW rotation
}
```

**Visual Representation:**
```
CCW Point Turn:          CW Point Turn:
Left: →                  Left: ←
      ↻                        ↺
Right: ←                 Right: →
```

---

## PWM Signal Generation

### What is PWM?

**PWM (Pulse Width Modulation)** controls motor speed by rapidly switching power on/off.

- **Frequency**: How many times per second the signal switches (Hz)
- **Duty Cycle**: Percentage of time signal is HIGH (0-100%)
- **Resolution**: Number of discrete duty cycle levels (8-bit = 0-255)

**Example:**
```
50% Duty Cycle (Medium Speed):
█████░░░░░█████░░░░░█████░░░░░
HIGH  LOW HIGH  LOW HIGH  LOW

100% Duty Cycle (Full Speed):
██████████████████████████████
HIGH (Always On)

25% Duty Cycle (Low Speed):
██░░░░░░░░██░░░░░░░░██░░░░░░░░
HIGH LOW   HIGH LOW   HIGH LOW
```

### ESP32 PWM Configuration

```cpp
// PWM Parameters
int freq = 16000;    // 16 kHz frequency (above human hearing)
int res = 8;         // 8-bit resolution (0-255 levels)
int channelL = 1;    // PWM channel for left motor
int channelR = 0;    // PWM channel for right motor
```

**Why 16 kHz?**
- High enough to be silent (no motor whine)
- Low enough for motor driver compatibility
- Smooth motor operation

**Setup PWM Channels:**
```cpp
ledcSetup(channelL, freq, res);  // Configure channel with frequency and resolution
ledcSetup(channelR, freq, res);

ledcAttachPin(lmdpwm, channelL); // Attach channel to GPIO pin
ledcAttachPin(rmdpwm, channelR);
```

**Generate PWM Signal:**
```cpp
ledcWrite(channelL, pwmValue);  // Write duty cycle (0-255)
ledcWrite(channelR, pwmValue);
```

---

## Motor Control Function

The `setMotorSpeeds()` function converts desired speed values into PWM signals and direction control.

### Function Signature:
```cpp
void setMotorSpeeds(int leftSpeed, int rightSpeed)
```

**Parameters:**
- `leftSpeed`: -255 to +255 (negative = reverse, positive = forward)
- `rightSpeed`: -255 to +255 (negative = reverse, positive = forward)

### Implementation Breakdown:

#### Step 1: Constrain Input Values
```cpp
leftSpeed = constrain(leftSpeed, -max_pwm, max_pwm);
rightSpeed = constrain(rightSpeed, -max_pwm, max_pwm);
```
Ensures values don't exceed safe limits.

#### Step 2: Calculate PWM Magnitude
```cpp
int pwmL = abs(leftSpeed);   // Get absolute value (0-255)
int pwmR = abs(rightSpeed);
```
PWM always needs positive values; direction is controlled separately.

#### Step 3: Write PWM Signals
```cpp
ledcWrite(channelL, (leftSpeed == 0) ? 0 : pwmL);
ledcWrite(channelR, (rightSpeed == 0) ? 0 : pwmR);
```
- If speed is 0, send 0 PWM (motor stops)
- Otherwise, send calculated PWM value

#### Step 4: Set Direction Pins
```cpp
digitalWrite(lmddir, leftSpeed >= 0 ? LOW : HIGH);
digitalWrite(rmddir, rightSpeed >= 0 ? LOW : HIGH);
```

**Direction Logic:**
- Positive speed → Direction pin LOW → Motor forward
- Negative speed → Direction pin HIGH → Motor reverse
- Zero speed → Motor stopped (PWM is 0)

**Note:** Direction pin logic (LOW=forward, HIGH=reverse) may vary by motor driver. Swap if motors run backward.

---

### 4-Wheel Configuration Differences

For 4-wheel robots, both motors on each side receive the same control signals:

```cpp
// Left side motors (both get same PWM and direction)
ledcAttachPin(lmdpwm1, channelL);
ledcAttachPin(lmdpwm2, channelL);  // Same channel as lmdpwm1

if (leftSpeed >= 0) {
    digitalWrite(lmddir1, HIGH);
    digitalWrite(lmddir2, HIGH);  // Same direction
} else {
    digitalWrite(lmddir1, LOW);
    digitalWrite(lmddir2, LOW);
}

// Right side motors (same concept)
ledcAttachPin(rmdpwm1, channelR);
ledcAttachPin(rmdpwm2, channelR);
```

---

## Configuration Parameters

### Critical Parameters to Tune:

#### 1. Speed Parameters
```cpp
int maxSpeed = 255;        // Maximum robot speed (0-255)
int baseSpeed = 90;        // Default cruising speed
int speedChangeRate = 6;   // Acceleration rate (higher = faster changes)
```

**Tuning Tips:**
- Lower `baseSpeed` for beginners or testing
- Increase `speedChangeRate` for more responsive acceleration
- Decrease `speedChangeRate` for smoother, less jerky motion

#### 2. Control Sensitivity
```cpp
int joystickDeadzone = 70;     // Ignore small joystick movements
float turnSensitivity = 0.55;  // Turn response (0.0-1.0)
```

**Tuning Tips:**
- Increase `joystickDeadzone` if robot drifts when joystick is centered
- Decrease `turnSensitivity` for wider, more controllable turns
- Increase `turnSensitivity` for tighter, more aggressive turns

#### 3. PWM Configuration
```cpp
int freq = 16000;  // PWM frequency (Hz)
int res = 8;       // PWM resolution (bits)
```

**Tuning Tips:**
- Keep `freq` between 10-20 kHz for most motor drivers
- Higher frequency = smoother but may overheat driver
- Lower frequency = audible whine but cooler operation

#### 4. Motor Calibration
```cpp
// For unbalanced motors (one side faster than other)
setMotorSpeeds(leftSp, rightSp * 0.955);  // Reduce right side by 4.5%
```

**Finding Calibration Value:**
1. Run robot straight forward at medium speed
2. If veers right → reduce right motor multiplier (< 1.0)
3. If veers left → reduce left motor multiplier or increase right (> 1.0)

---

## Emergency Stop

**Safety Feature:** Pressing L1 + R1 simultaneously stops all motors immediately.

```cpp
if ((ctl->buttons() & 0x0030) == 0x0030) {  // Both L1 and R1
    setMotorSpeeds(0, 0);
    Serial.println("EMERGENCY STOP ACTIVATED");
    return;  // Exit function immediately
}
```

**Button Bitmask:**
- `0x0010`: L1 button
- `0x0020`: R1 button
- `0x0030`: Both buttons (bitwise OR)

---

## Complete Data Flow Example

Let's trace what happens when you push the left joystick forward and right joystick to the right:

### Input Values:
```
yAxis (forward): +400 (out of ±512)
xAxis (right): +300 (out of ±512)
currentSpeed: 100
```

### Step-by-Step Processing:

1. **Deadzone Check:**
```cpp
abs(400) > 70 → Keep yAxis = 400
abs(300) > 70 → Keep xAxis = 300
```

2. **Normalization:**
```cpp
yAxis = 400 / 512.0 = 0.78
xAxis = 300 / 512.0 = 0.59
```

3. **Turn Sensitivity:**
```cpp
xAxis = 0.59 * 0.55 = 0.32
```

4. **Scaling by Current Speed:**
```cpp
scale = 100 / 255 = 0.39
forward = 0.78 * 0.39 * 255 = 78
turn = 0.32 * 0.39 * 255 = 32
```

5. **Differential Drive Mixing:**
```cpp
leftSpeed = 78 - 32 = 46    // Slower (turning right)
rightSpeed = 78 + 32 = 110  // Faster (turning right)
```

6. **PWM Generation:**
```cpp
leftPWM = 46, leftDir = LOW (forward)
rightPWM = 110, rightDir = LOW (forward)
```

7. **Result:** Robot moves forward while turning right, with right motor running faster than left.

---

## Troubleshooting

### Problem: Robot doesn't move

**Possible Causes:**
1. Motor driver not powered
2. ESP32 not connected to motor driver
3. Wrong pin configuration
4. Controller not paired

**Solutions:**
```cpp
// Check serial output for controller connection
Serial.printf("Controller connected: %d\n", ctl->isConnected());

// Test motors directly
setMotorSpeeds(100, 100);  // Add in setup() to test
```

---

### Problem: Robot moves when joystick is centered

**Cause:** Joystick deadzone too small

**Solution:**
```cpp
int joystickDeadzone = 70;  // Increase this value (try 100-150)
```

---

### Problem: Robot veers to one side when going straight

**Cause:** Motors have different speeds

**Solution:** Add calibration multiplier:
```cpp
// If veers right, reduce right motor speed
setMotorSpeeds(leftSp, rightSp * 0.95);  // Start with 0.95, adjust as needed

// If veers left, reduce left motor speed
setMotorSpeeds(leftSp * 0.95, rightSp);
```

---

### Problem: Motors whine/make noise

**Cause:** PWM frequency too low

**Solution:**
```cpp
int freq = 20000;  // Increase frequency above hearing range
```

**Note:** Very high frequencies (>25kHz) may cause motor driver overheating.

---

### Problem: Jerky acceleration

**Cause:** Speed change rate too high

**Solution:**
```cpp
int speedChangeRate = 3;  // Decrease for smoother acceleration (try 2-4)
```

---

### Problem: Turns are too aggressive

**Cause:** Turn sensitivity too high

**Solution:**
```cpp
float turnSensitivity = 0.3;  // Decrease for wider turns (try 0.3-0.5)
```

---

### Problem: Not enough power/speed

**Possible Causes:**
1. Battery voltage low
2. Speed limits set too low
3. PWM duty cycle limited

**Solutions:**
```cpp
// Increase maximum speed
int maxSpeed = 255;  // Ensure this is at maximum

// Check battery voltage
Serial.println(analogRead(BATTERY_PIN));  // Add voltage monitoring

// Remove PWM mapping limits if present
int pwmL = abs(leftSpeed);  // Use full range, not mapped
```

---

### Problem: Motor driver overheats

**Causes:**
1. PWM frequency too high
2. Current draw too high
3. Insufficient cooling

**Solutions:**
```cpp
// Reduce PWM frequency
int freq = 12000;  // Lower frequency (try 10-15 kHz)

// Limit maximum speed to reduce current
int maxSpeed = 200;  // Reduce from 255

// Add heatsink to motor driver
// Check motor current rating vs driver rating
```

---

### Problem: Controller won't connect

**Solutions:**
```cpp
// In setup(), forget old pairings
BP32.forgetBluetoothKeys();

// Check controller is in pairing mode:
// PS4: Hold Share + PS button until light flashes
// Xbox: Hold pairing button until light flashes

// Check serial monitor for pairing status
Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", 
              bd[0], bd[1], bd[2], bd[3], bd[4], bd[5]);
```

---

### Problem: Robot loses connection during operation

**Causes:**
1. Controller battery low
2. ESP32 power issues
3. Bluetooth interference

**Solutions:**
```cpp
// Add connection check in loop
if (!ctl->isConnected()) {
    setMotorSpeeds(0, 0);  // Stop when disconnected
    Serial.println("Controller disconnected!");
}

// Ensure stable ESP32 power supply
// Keep controller within 10 meters
// Avoid operation near WiFi routers or microwaves
```

---

## Advanced Customization

### Adding Speed Profiles

```cpp
enum SpeedProfile { SLOW, MEDIUM, FAST };
SpeedProfile currentProfile = MEDIUM;

void setSpeedProfile(SpeedProfile profile) {
    switch(profile) {
        case SLOW:
            maxSpeed = 150;
            baseSpeed = 60;
            break;
        case MEDIUM:
            maxSpeed = 200;
            baseSpeed = 90;
            break;
        case FAST:
            maxSpeed = 255;
            baseSpeed = 120;
            break;
    }
}

// Toggle with button press
if (ctl->buttons() & 0x0100) {  // Select button
    currentProfile = (currentProfile + 1) % 3;
    setSpeedProfile(currentProfile);
}
```

---

### Adding Battery Monitoring

```cpp
#define BATTERY_PIN 34  // ADC pin

float getBatteryVoltage() {
    int rawValue = analogRead(BATTERY_PIN);
    // Assuming voltage divider for 12V battery
    float voltage = (rawValue / 4095.0) * 3.3 * 4;  // Adjust multiplier
    return voltage;
}

void checkBattery() {
    float voltage = getBatteryVoltage();
    if (voltage < 10.5) {  // Low battery threshold
        Serial.println("WARNING: Low battery!");
        // Optionally reduce max speed
        maxSpeed = 150;
    }
}
```

---

### Adding Rotation Lock (Straight Line Mode)

```cpp
bool straightLineMode = false;

// Toggle with button
if (ctl->buttons() & 0x0200) {  // Start button
    straightLineMode = !straightLineMode;
}

// In differential drive section
if (straightLineMode) {
    xAxis = 0;  // Ignore turn input
}
```

---

### Adding Speed Display (via Serial)

```cpp
void displaySpeed() {
    Serial.print("Speed: [");
    int bars = map(currentSpeed, 0, maxSpeed, 0, 20);
    for (int i = 0; i < 20; i++) {
        Serial.print(i < bars ? "█" : "░");
    }
    Serial.printf("] %d/%d\n", currentSpeed, maxSpeed);
}
```

---

## Understanding Motor Driver Compatibility

Different motor drivers may require different PWM ranges:

```cpp
// SmartElex 15D driver (example)
int pwmMin = 20;   // Below this, motor doesn't move
int pwmMax = 240;  // Above this, driver may not respond

// Map 0-255 to working range
int pwmL = map(abs(leftSpeed), 0, 255, pwmMin, pwmMax);
int pwmR = map(abs(rightSpeed), 0, 255, pwmMin, pwmMax);

// MDD10A driver
// Usually accepts full 0-255 range
int pwmL = abs(leftSpeed);
int pwmR = abs(rightSpeed);
```

**Finding Your Driver's Range:**
1. Set one motor to fixed speed (start with 50)
2. Gradually increase until motor starts moving (note minimum)
3. Increase to maximum and check if motor speed still increases (note maximum)
4. Map PWM values to this working range

---

## Performance Optimization

### Reducing Loop Delay

```cpp
void loop() {
    // Original
    if (BP32.update()) {
        processControllers();
    }
    vTaskDelay(1);  // 1ms delay
    
    // Optimized for faster response
    BP32.update();
    processControllers();
    // Remove delay or reduce to 0
}
```

**Note:** Removing delay may cause higher CPU usage and faster battery drain.

---

### Pre-calculating Values

```cpp
// Instead of calculating every loop
float scale = (float)currentSpeed / (float)maxSpeed;

// Pre-calculate in setup if values don't change
float scaleConstant = (float)baseSpeed / (float)maxSpeed;
```

---

## Code Comments Reference

### Important Code Sections:

1. **Pin Definitions** (Lines 4-11): GPIO pin assignments
2. **PWM Configuration** (Lines 11-15): Frequency and resolution
3. **Speed Control Variables** (Lines 17-24): Tuning parameters
4. **Bluepad32 Callbacks** (Lines 31-65): Controller connection handling
5. **Controller Input Processing** (Lines 78-84): Reading controller data
6. **Differential Drive Logic** (Lines 173-180): Motor speed calculation
7. **PWM Generation** (Lines 221-242): Motor control function
8. **Main Loop** (Lines 245-250): Continuous execution

---

## Summary

This ESP32 robot hockey controller implements a complete control system:

1. **Input:** Bluetooth controller (Bluepad32 library)
2. **Processing:** Differential drive mathematics
3. **Output:** PWM signals to motor drivers

**Key Concepts:**
- PWM controls motor speed through duty cycle
- Direction pins control motor rotation direction
- Differential drive enables steering by varying left/right motor speeds
- Deadzone eliminates controller drift
- Speed ramping prevents jerky movement
- Turn sensitivity enables precise control

**Customization Points:**
- Speed limits (maxSpeed, baseSpeed)
- Control sensitivity (turnSensitivity, joystickDeadzone)
- PWM parameters (frequency, resolution)
- Motor calibration (speed multipliers)

**Safety Features:**
- Emergency stop (L1 + R1)
- Auto-stop on controller disconnect
- PWM value constraints
- Input value validation

This system provides smooth, responsive control suitable for competitive robot hockey while remaining simple to understand and customize.

---

## Additional Resources

### Bluepad32 Library:
- GitHub: https://github.com/ricardoquesada/bluepad32
- Documentation: https://gitlab.com/ricardoquesada/bluepad32

### ESP32 PWM:
- ESP32 LEDC Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html

### Motor Control Theory:
- Differential Drive Kinematics
- PWM Motor Control Basics
- H-Bridge Motor Driver Operation

---

**Document Version:** 1.0  
**Last Updated:** 2024  
**Applicable Code Versions:** 2-wheel and 4-wheel configurations  
**Target Hardware:** ESP32 + SmartElex 15D / MDD10A Motor Drivers
