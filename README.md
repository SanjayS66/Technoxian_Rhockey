# Technoxian Robohockey Bot - ESP32 Control System

Repository to store the Arduino sketches of robohockey Bot to be used in technoxian competition.

## 🤖 Overview

This repository contains ESP32-based Arduino sketches for controlling a differential drive robohockey bot using Bluetooth gamepad input. The bot receives joystick commands wirelessly and translates them into PWM signals for precise motor control.

## 🎮 System Architecture

### How It Works

1. **Bluetooth Connection**: The ESP32 connects to a gamepad controller (PS4/Xbox compatible) via Bluetooth using the Bluepad32 library
2. **Input Processing**: Joystick axes, triggers, and buttons are read from the controller
3. **Differential Drive Calculation**: The code converts joystick inputs into left/right motor speeds
4. **PWM Signal Generation**: Motor speeds are converted to PWM signals (0-255) with directional control
5. **Motor Control**: PWM signals are sent to motor drivers to control the robot's movement

### Differential Drive Logic

The robot uses a two-wheel differential drive system where:
- **Forward/Backward**: Both motors run at the same speed in the same direction
- **Turning**: Motors run at different speeds (or opposite directions) to create rotation
- **Point Turns**: Motors run at equal speeds in opposite directions for in-place rotation

## 📁 Repository Structure

```
Technoxian_Rhockey/
├── rhockey_final/
│   ├── new_2wheel_final/        # Main 2-wheel bot sketch
│   └── gk_final/                # Goalkeeper bot variant
├── 2_wheel/                     # Early 2-wheel implementations
├── 4_wheel/                     # 4-wheel drive experiments
├── smart_elex_code/            # Smart Elex driver specific code
├── wifi_disabled code/         # Bluetooth-only versions
├── misc codes/                 # Testing and utility sketches
└── HockeyBot Inventory.txt     # Hardware inventory
```

## 🔧 Hardware Configuration

### Pin Definitions(can be changed accordingly)

| Component | ESP32 Pin | Description |
|-----------|-----------|-------------|
| Right Motor PWM | GPIO 33 | PWM signal for right motor speed |
| Right Motor DIR | GPIO 32 | Direction control for right motor |
| Left Motor PWM | GPIO 26 | PWM signal for left motor speed |
| Left Motor DIR | GPIO 25 | Direction control for left motor |
| LED | GPIO 2 | Status/debug LED |

### PWM Configuration

- **Frequency**: 16 kHz
- **Resolution**: 8-bit (0-255)
- **Channel 0**: Right motor
- **Channel 1**: Left motor

## 🎯 Control Features

### Main Features (`new_2wheel_final.ino`)

#### 1. **Joystick Control**
- **Left Analog Y-axis**: Forward/backward movement
- **Right Analog X-axis**: Turning left/right
- Configurable deadzone (default: 70) to prevent drift
- Turn sensitivity adjustment (default: 0.55) for smoother control

#### 2. **Speed Control**
- **R2 Trigger (Throttle)**: Accelerate from baseSpeed to maxSpeed
- **L2 Trigger (Brake)**: Decelerate below baseSpeed
- **Base Speed**: 90 (default cruising speed)
- **Max Speed**: 255 (full throttle)
- **Speed Change Rate**: 6 (prevents jerky movements)

#### 3. **Special Maneuvers**
- **X/□ Button**: Counter-clockwise point turn
- **B/○ Button**: Clockwise point turn
- **L1 + R1**: Emergency stop (both shoulder buttons)

## 💻 Code Structure

### Main Functions

```cpp
void onConnectedController(ControllerPtr ctl)
```
Callback when a gamepad connects via Bluetooth. Restricted to 1 controller maximum.

```cpp
void processGamepad(ControllerPtr ctl)
```
Core logic that processes controller inputs and implements differential drive calculations.

```cpp
void setMotorSpeeds(int leftSpeed, int rightSpeed)
```
Sets PWM duty cycle and direction pins for both motors. Handles speed constraints and direction control.

```cpp
void dumpGamepad(ControllerPtr ctl)
```
Debug function that prints controller values to Serial monitor.

### Differential Drive Algorithm

```cpp
// Normalize joystick inputs to -1.0 to +1.0
yAxis = yAxis / 512.0;  // Forward/backward
xAxis = xAxis / 512.0;  // Turning

// Apply turn sensitivity
xAxis *= turnSensitivity;

// Calculate motor speeds
float scale = (float)currentSpeed / (float)maxSpeed;
int forward = (int)(yAxis * scale * maxSpeed);
int turn = (int)(xAxis * scale * maxSpeed);

int leftSpeed  = constrain(forward - turn, -max_pwm, max_pwm);
int rightSpeed = constrain(forward + turn, -max_pwm, max_pwm);
```

## 📚 Dependencies

### Required Libraries

1. **Bluepad32**: Bluetooth gamepad support for ESP32
   - Install via Arduino Library Manager or from [GitHub](https://github.com/ricardoquesada/bluepad32)
   - Provides `ControllerPtr`, `BP32`, and gamepad connection management

### Installation

```bash
# Install ESP32 board support in Arduino IDE
# Boards Manager URL: https://dl.espressif.com/dl/package_esp32_index.json

# Install Bluepad32 library
# Library Manager -> Search "Bluepad32" -> Install
```

## 🚀 Getting Started

### Upload Instructions

1. **Install Arduino IDE** (1.8.x or 2.x)
2. **Add ESP32 Board Support**:
   - File → Preferences → Additional Board URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
3. **Install Bluepad32 Library**:
   - Sketch → Include Library → Manage Libraries
   - Search "Bluepad32" → Install
4. **Open Sketch**:
   - Open `rhockey_final/new_2wheel_final/new_2wheel_final.ino`
5. **Configure Board**:
   - Tools → Board → ESP32 Dev Module
   - Tools → Upload Speed → 115200
6. **Connect ESP32** via USB
7. **Upload** the sketch

### Pairing Controller

**Note:** For PS4 controllers, you must first set the ESP32's Bluetooth MAC address as the master controller using the [SixAxisPairTool](https://github.com/user-none/sixaxispairtool) before pairing. 

1. Upload code to ESP32
2. Open Serial Monitor (115200 baud)
3. Put controller in pairing mode: 
   - **PS4**: Hold SHARE + PS button until the light bar flashes rapidly
   - **Xbox**: Hold the pairing button on top of the controller
4. ESP32 should detect and connect automatically
5. Serial monitor will show: `CALLBACK: Controller is connected`

## ⚙️ Configuration & Tuning

### Adjustable Parameters

```cpp
// Speed tuning
int maxSpeed = 255;           // Maximum speed (0-255)
int baseSpeed = 90;           // Cruising speed
int speedChangeRate = 6;      // Acceleration smoothness

// Control tuning
int joystickDeadzone = 70;    // Deadzone threshold
float turnSensitivity = 0.55; // Turn response (0.0-1.0)
```


## 🏒 Competition Variants

### `new_2wheel_final.ino`
- Optimized for 2-wheel striker bot
- Aggressive acceleration (speedChangeRate: 6)
- Balanced turn sensitivity (0.55)
- Lower base speed (90) for better control

### `gk_final.ino`
- Goalkeeper variant
- Higher base speed (150) for quick lateral movement
- Same differential drive logic
- Optimized for defensive positioning

## 🛡️ Safety Features

1. **Emergency Stop**: L1+R1 buttons immediately halt all motors
2. **Disconnect Protection**: Motors stop when controller disconnects
3. **Speed Ramping**: Gradual acceleration prevents mechanical stress
4. **Brake Priority**: Brake trigger overrides throttle input
5. **PWM Constraints**: Motor speeds clamped to safe ranges

## 📊 Technical Specifications

- **Microcontroller**: ESP32
- **Communication**: Bluetooth Classic (Bluepad32)
- **Controller Support**: PS4, Xbox One, generic HID gamepads
- **PWM Frequency**: 16 kHz
- **PWM Resolution**: 8-bit (256 levels)
- **Update Rate**: ~1ms loop delay
- **Max Controllers**: 1 (configured)

## 🤝 Contributing

This is a competition project for Technoxian Robohockey. Feel free to:
- Report issues with hardware compatibility
- Suggest control improvements
- Share tuning parameters for different motor/driver combinations

## 📝 License

Open source for educational and competition purposes.

## 🏆 Competition

**Event**: Technoxian Robohockey Championship  
**Category**: RC Robohockey  
**Bot Type**: Differential Drive Striker & Goalkeeper

---

**Author**: SanjayS66  
**Last Updated**: January 2026

**Current Date and Time (UTC - YYYY-MM-DD HH:MM:SS formatted)**: 2026-01-04 05:23:19  
**Current User's Login**: SanjayS66
