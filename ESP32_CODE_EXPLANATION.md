# ESP32 Robot Hockey Controller - Code Explanation

## Table of Contents
1. [Overview](#overview)
2. [Bluepad32 Controller Integration](#bluepad32-controller-integration)
3. [Controller Input Processing](#controller-input-processing)
4. [PWM Signal Generation & Motor Control](#pwm-signal-generation--motor-control)
5. [Differential Drive Logic](#differential-drive-logic)
6. [Configuration Parameters](#configuration-parameters)
7. [Troubleshooting](#troubleshooting)

---

## Overview

This ESP32-based robot hockey controller uses **Bluepad32** library to receive wireless input from Bluetooth game controllers (PS4/Xbox) and translates those inputs into **PWM (Pulse Width Modulation) signals** to control DC motors through motor drivers using **differential drive** mechanics.

**Key Features:**
- Bluetooth controller support (PS4, Xbox)
- Differential drive control for smooth turning
- Dynamic speed control with acceleration/braking
- Point turn capability for in-place rotation
- Emergency stop functionality (L1 + R1)
- Customizable speed and sensitivity parameters

**Processing Pipeline:**
```
Controller Input → Deadzone Filter → Normalize → Speed Calculation → 
Turn Sensitivity → Differential Drive Mix → PWM Signals → Motors
```

---

## Bluepad32 Controller Integration

**Bluepad32** is an Arduino library that enables Bluetooth controller support on ESP32. It handles pairing, connection, and data reception from PS4/Xbox controllers.

**Key Callbacks:**
```cpp
void onConnectedController(ControllerPtr ctl) {
    myControllers[i] = ctl;  // Store controller pointer
}

void onDisconnectedController(ControllerPtr ctl) {
    setMotorSpeeds(0, 0);    // Stop motors for safety
    myControllers[i] = nullptr;
}
```

**Setup:**
```cpp
void setup() {
    // Configure PWM for motor control
    ledcSetup(channelL, freq, res);  // Left motor
    ledcSetup(channelR, freq, res);  // Right motor
    ledcAttachPin(lmdpwm, channelL);
    ledcAttachPin(rmdpwm, channelR);
    
    // Setup Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    if (BP32.update()) {
        processControllers();  // Process inputs
    }
    vTaskDelay(1);
}
```

---

## Controller Input Processing

**Reading Inputs:**
```cpp
// Joysticks (analog: -511 to +512)
float yAxis = (float)(ctl->axisY());   // Forward/backward
float xAxis = (float)(ctl->axisRX());  // Left/right turning

// Triggers (analog: 0-1023)
int accel = ctl->throttle();  // Right trigger - increase speed
int brake = ctl->brake();     // Left trigger - decrease speed

// Buttons (digital)
bool ccwpturn = ctl->buttons() & 0x0004;  // X/Square - CCW point turn
bool cwpturn = ctl->buttons() & 0x0002;   // B/Circle - CW point turn
bool emergencyStop = (ctl->buttons() & 0x0030) == 0x0030;  // L1+R1
```

**Deadzone & Normalization:**

Controller joysticks may not return exactly 0 when centered. Deadzone eliminates drift.

```cpp
int joystickDeadzone = 70;

// Apply deadzone
if (abs(yAxis) < joystickDeadzone) yAxis = 0;
if (abs(xAxis) < joystickDeadzone) xAxis = 0;

// Normalize to -1.0...+1.0 range
yAxis = constrain(yAxis / 512.0, -1.0, 1.0);
xAxis = constrain(xAxis / 512.0, -1.0, 1.0);
```

**Speed Control:**

```cpp
int baseSpeed = 90;          // Default cruising speed
int maxSpeed = 255;          // Maximum speed (full throttle)
int currentSpeed = baseSpeed;
int speedChangeRate = 6;     // Acceleration rate

// Throttle increases speed
if (accel > 50) {
    int targetSpeed = map(accel, 0, 1023, baseSpeed, maxSpeed);
    currentSpeed = min(currentSpeed + speedChangeRate, targetSpeed);
}
// Brake decreases speed
else if (brake > 50) {
    int brakeAmount = map(brake, 0, 1023, 0, baseSpeed);
    int targetSpeed = max(baseSpeed - brakeAmount, 0);
    currentSpeed = max(currentSpeed - speedChangeRate, targetSpeed);
}
```

**Turn Sensitivity:**

Reduces turn response for better control at high speeds.

```cpp
float turnSensitivity = 0.55;  // 0.0-1.0 (lower = wider turns)
xAxis *= turnSensitivity;
```

---

## PWM Signal Generation & Motor Control

**PWM (Pulse Width Modulation)** controls motor speed by rapidly switching power on/off.

- **Duty Cycle**: 0% = stopped, 50% = half speed, 100% = full speed
- **Frequency**: 16 kHz (above human hearing, prevents motor whine)
- **Resolution**: 8-bit (0-255 levels)

**ESP32 Configuration:**
```cpp
int freq = 16000;  // PWM frequency
int res = 8;       // 8-bit resolution (0-255)

ledcSetup(channelL, freq, res);
ledcSetup(channelR, freq, res);
ledcAttachPin(lmdpwm, channelL);
ledcAttachPin(rmdpwm, channelR);
```

**Motor Control Function:**

```cpp
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // leftSpeed/rightSpeed: -255 to +255 (negative = reverse)
    
    // Step 1: Constrain values
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Step 2: Extract PWM magnitude (always positive)
    int pwmL = abs(leftSpeed);
    int pwmR = abs(rightSpeed);
    
    // Step 3: Write PWM signals
    ledcWrite(channelL, (leftSpeed == 0) ? 0 : pwmL);
    ledcWrite(channelR, (rightSpeed == 0) ? 0 : pwmR);
    
    // Step 4: Set direction pins
    digitalWrite(lmddir, leftSpeed >= 0 ? LOW : HIGH);  // LOW=forward, HIGH=reverse
    digitalWrite(rmddir, rightSpeed >= 0 ? LOW : HIGH);
}
```

**Note:** Direction pin logic (LOW=forward, HIGH=reverse) may vary by motor driver.

---

## Differential Drive Logic

Differential drive controls robot direction by varying left and right motor speeds.

**Mathematical Mixing:**
```cpp
// Scale by current speed
float scale = (float)currentSpeed / (float)maxSpeed;
int forward = (int)(yAxis * scale * maxSpeed);  // Forward/backward
int turn = (int)(xAxis * scale * maxSpeed);     // Turn rate

// Mix to get individual motor speeds
int leftSpeed  = forward - turn;   // Left motor
int rightSpeed = forward + turn;   // Right motor

// Constrain to valid range
leftSpeed  = constrain(leftSpeed, -255, 255);
rightSpeed = constrain(rightSpeed, -255, 255);

setMotorSpeeds(leftSpeed, rightSpeed);
```

**Movement Examples:**

| Forward | Turn | Left Motor | Right Motor | Result |
|---------|------|------------|-------------|--------|
| 100     | 0    | 100        | 100         | Straight forward |
| 100     | 50   | 50         | 150         | Forward + right turn |
| 100     | -50  | 150        | 50          | Forward + left turn |
| 0       | 100  | -100       | 100         | Spin right in place |
| -100    | 0    | -100       | -100        | Straight backward |

**Point Turn:** For in-place rotation, press X/Square (CCW) or B/Circle (CW) buttons:
```cpp
if (ccwpturn) {
    setMotorSpeeds(currentSpeed, -currentSpeed);  // CCW rotation
}
if (cwpturn) {
    setMotorSpeeds(-currentSpeed, currentSpeed);  // CW rotation
}
```

---

## Configuration Parameters

**Key Tunable Parameters:**

```cpp
// Speed Control
int maxSpeed = 255;              // Maximum speed (0-255)
int baseSpeed = 90;              // Default cruising speed
int speedChangeRate = 6;         // Acceleration rate

// Control Sensitivity  
int joystickDeadzone = 70;       // Ignore small joystick movements
float turnSensitivity = 0.55;    // Turn response (0.0-1.0)

// PWM Settings
int freq = 16000;                // PWM frequency (10-20 kHz recommended)
int res = 8;                     // 8-bit resolution (0-255)
```

**Tuning Guide:**
- **baseSpeed**: Lower for beginners, higher for experienced drivers
- **speedChangeRate**: Higher = more responsive, lower = smoother
- **joystickDeadzone**: Increase if robot drifts when centered
- **turnSensitivity**: Lower = wider turns, higher = tighter turns
- **freq**: Keep 10-20 kHz (higher = quieter but may overheat driver)

**Motor Calibration (if robot veers to one side):**
```cpp
setMotorSpeeds(leftSp, rightSp * 0.95);  // If veers right, reduce right by 5%
setMotorSpeeds(leftSp * 0.95, rightSp);  // If veers left, reduce left by 5%
```

**Emergency Stop:** L1 + R1 buttons stop all motors immediately.

---

## Troubleshooting

**Common Issues and Solutions:**

| Problem | Cause | Solution |
|---------|-------|----------|
| Robot doesn't move | Controller not paired or motor driver unpowered | Check serial monitor for "Controller connected". Test with `setMotorSpeeds(100, 100)` in setup() |
| Drifts when centered | Deadzone too small | Increase `joystickDeadzone` to 100-150 |
| Veers to one side | Motor imbalance | Add calibration: `setMotorSpeeds(leftSp, rightSp * 0.95)` |
| Motor whine/noise | PWM frequency too low | Increase `freq` to 18000-20000 Hz |
| Jerky movement | Speed change rate too high | Decrease `speedChangeRate` to 2-4 |
| Aggressive turns | Turn sensitivity too high | Decrease `turnSensitivity` to 0.3-0.4 |
| Low power | Battery low or speed limits | Check battery voltage, set `maxSpeed = 255` |
| Driver overheats | PWM freq too high or overcurrent | Lower `freq` to 12000 Hz, add heatsink |
| Won't pair | Old pairing data | Add `BP32.forgetBluetoothKeys()` in setup() |
| Loses connection | Low battery or interference | Charge controller, keep within 10m range |

**Pairing Controllers:**
- **PS4:** Hold Share + PS until light flashes rapidly
- **Xbox:** Hold small pairing button until light flashes

---

## Summary


This ESP32 robot hockey controller system provides:

1. **Input:** Bluetooth controller via Bluepad32 library
2. **Processing:** Differential drive mathematics with configurable parameters
3. **Output:** PWM signals (16 kHz, 8-bit) to motor drivers

**Key Features:**
- Deadzone filtering eliminates controller drift
- Speed ramping prevents jerky acceleration
- Turn sensitivity enables precise control
- Point turn for in-place rotation
- Emergency stop (L1+R1) for safety

**Main Code Flow:**
```
Controller → Deadzone → Normalize → Speed Calc → 
Turn Sensitivity → Differential Mix → PWM Output → Motors
```

This provides smooth, responsive control for competitive robot hockey.

---

## Additional Resources

- **Bluepad32:** https://github.com/ricardoquesada/bluepad32
- **ESP32 LEDC (PWM):** https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html

---

**Document Version:** 1.0  
**Hardware:** ESP32 + SmartElex 15D / MDD10A Motor Drivers
