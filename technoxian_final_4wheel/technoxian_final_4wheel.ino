#include <Bluepad32.h>

int rmdpwm1 = 35;
int rmdpwm2 = 13;
int rmddir1 = 32;
int rmddir2 = 14;
int lmdpwm1 = 19;
int lmdpwm2 = 16;
int lmddir1 = 18;
int lmddir2 = 4;
 


//PWM VARIABLES *if you are using ledc functio for custom pwm*
int freq = 5000;           // PWM frequency 
int res = 8;        // 8-bit resolution (0-255)

// PWM channel assignments(timer that helps generates pwm signal)
int channelR = 0;        // PWM channel for right motors
int channelL = 1;        // PWM channel for left motors
int max_pwm = 255;
int maxSpeed = 255;
int baseSpeed = 150;
int currentSpeed = baseSpeed;
int speedChangeRate = 8;
int joystickDeadzone = 15; 

//******************CODE FROM EXAMPLE SKETCH(NOTHING TO CHANGE)*********************
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

//**********************PRINT INPUT VALUES ON SERIAL MONITOR**********************

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

// void dumpMouse(ControllerPtr ctl) {
//     Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
//                    ctl->index(),        // Controller Index
//                    ctl->buttons(),      // bitmask of pressed buttons
//                    ctl->scrollWheel(),  // Scroll Wheel
//                    ctl->deltaX(),       // (-511 - 512) left X Axis
//                    ctl->deltaY()        // (-511 - 512) left Y axis
//     );
// }

// void dumpKeyboard(ControllerPtr ctl) {
//     static const char* key_names[] = {
//         // clang-format off
//         // To avoid having too much noise in this file, only a few keys are mapped to strings.
//         // Starts with "A", which is offset 4.
//         "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
//         "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
//         // Special keys
//         "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
//         "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
//         // Function keys
//         "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
//         // Cursors and others
//         "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
//         "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
//         // clang-format on
//     };
//     static const char* modifier_names[] = {
//         // clang-format off
//         // From 0xe0 to 0xe7
//         "Left Control", "Left Shift", "Left Alt", "Left Meta",
//         "Right Control", "Right Shift", "Right Alt", "Right Meta",
//         // clang-format on
//     };
//     Serial.printf("idx=%d, Pressed keys: ", ctl->index());
//     for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
//         if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
//             const char* keyName = key_names[key-4];
//             Serial.printf("%s,", keyName);
//        }
//     }
//     for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
//         if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
//             const char* keyName = modifier_names[key-0xe0];
//             Serial.printf("%s,", keyName);
//         }
//     }
//     Console.printf("\n");
// }
//
// void dumpBalanceBoard(ControllerPtr ctl) {
//     Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
//                    ctl->index(),        // Controller Index
//                    ctl->topLeft(),      // top-left scale
//                    ctl->topRight(),     // top-right scale
//                    ctl->bottomLeft(),   // bottom-left scale
//                    ctl->bottomRight(),  // bottom-right scale
//                    ctl->temperature()   // temperature: used to adjust the scale value's precision
//     );
// }

void processGamepad(ControllerPtr ctl) {
    int yInput = int(ctl->axisY()) ; 
    int xInput = int(ctl->axisX()) ;
    int accelerate = int(ctl->throttle());
    int brake = int(ctl->brake());
    bool pturn = bool(ctl->buttons() & 0x0200);

    int yAxis = map(yInput,-511,512,-255,255);
    int xAxis = map(xInput,-511,512,-255,255);


    if (abs(yAxis) < joystickDeadzone) yAxis = 0;
    if (abs(xAxis) < joystickDeadzone) xAxis = 0;

    if (accelerate > 15 /*check the printed values to set this deadzone*/ ) {
        int targetSpeed = map(accelerate, 0, 1023, baseSpeed, maxSpeed);   //turn the acceleration input form trigger input range to motor speed range
                                                                        //map(input value,input range,output range)
        if (currentSpeed < targetSpeed) {
            currentSpeed = min(currentSpeed + speedChangeRate, targetSpeed);   //increase speed stepwise to reduce jerk from sudden increase
        } else {
             currentSpeed = targetSpeed;
        }
    } else if (brake > 15  /*check the printed values to set this deadzone*/  ) {
    
    int brakeIntensity = map(brake, 0, 1023, 0, baseSpeed);      ///turn the brake input form trigger input range to motor speed range
    int targetSpeed = max(baseSpeed - brakeIntensity, 0);  

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        if (currentSpeed > targetSpeed) {
            currentSpeed = max(currentSpeed - speedChangeRate * 2, targetSpeed); // Faster braking   //decrease speed stepwise to reduce jerk from sudden decrease
        } else {
            currentSpeed = targetSpeed;
        }
    } 
    else if(pturn){
        setMotorSpeeds(maxSpeed/2,-maxSpeed/2); //set speed after testing..full speed might be jerky...
        return;
    }
    else {
    // No trigger input - return to base speed
        if (currentSpeed < baseSpeed) {
            currentSpeed = min(currentSpeed + speedChangeRate, baseSpeed);
        } else if (currentSpeed > baseSpeed) {
            currentSpeed = max(currentSpeed - speedChangeRate, baseSpeed);
        }
  }
  
// Calculate differential drive speeds using the combined formula
int xMapped = map(xAxis, -511, 512, -currentSpeed, currentSpeed);
int yMapped = map(yAxis, -511, 512, -currentSpeed, currentSpeed);
    
// Apply differential drive formula
int leftSpeed = constrain(yMapped + xMapped, -max_pwm, max_pwm);
int rightSpeed = constrain(yMapped - xMapped, -max_pwm, max_pwm);

   setMotorSpeeds(leftSpeed, rightSpeed);
  
   dumpGamepad(ctl);
}


// void processMouse(ControllerPtr ctl) {
//     // This is just an example.
//     if (ctl->scrollWheel() > 0) {
//         // Do Something
//     } else if (ctl->scrollWheel() < 0) {
//         // Do something else
//     }

//     // See "dumpMouse" for possible things to query.
//     dumpMouse(ctl);
// }

// void processKeyboard(ControllerPtr ctl) {
//     if (!ctl->isAnyKeyPressed())
//         return;

//     // This is just an example.
//     if (ctl->isKeyPressed(Keyboard_A)) {
//         // Do Something
//         Serial.println("Key 'A' pressed");
//     }

//     // Don't do "else" here.
//     // Multiple keys can be pressed at the same time.
//     if (ctl->isKeyPressed(Keyboard_LeftShift)) {
//         // Do something else
//         Serial.println("Key 'LEFT SHIFT' pressed");
//     }

//     // Don't do "else" here.
//     // Multiple keys can be pressed at the same time.
//     if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
//         // Do something else
//         Serial.println("Key 'Left Arrow' pressed");
//     }

//     // See "dumpKeyboard" for possible things to query.
//     dumpKeyboard(ctl);
// }

// void processBalanceBoard(ControllerPtr ctl) {
//     // This is just an example.
//     if (ctl->topLeft() > 10000) {
//         // Do Something
//     }

//     // See "dumpBalanceBoard" for possible things to query.
//     dumpBalanceBoard(ctl);
// }

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                //processMouse(myController);
            } else if (myController->isKeyboard()) {        //Selecting whatever is connnected
                //processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                //processBalanceBoard(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {

    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

      // Set up PWM channels
    ledcSetup(channelL, freq, res);   //(channel(0-15),freq,res)
    ledcSetup(channelR, freq, res);

  
    // Attach PWM channels to pins
    ledcAttachPin(rmdpwm1,channelR);    
    ledcAttachPin(rmdpwm2,channelR);
    ledcAttachPin(lmdpwm1,channelL);
    ledcAttachPin(lmdpwm2, channelL);


    pinMode(rmddir1,OUTPUT);
    pinMode(rmddir2,OUTPUT);
    pinMode(lmddir1,OUTPUT);
    pinMode(lmddir2,OUTPUT);



    // Initialize motors to stopped state
    setMotorSpeeds(0, 0);


    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
}


void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  
  leftSpeed = constrain(leftSpeed, -max_pwm, max_pwm);
  rightSpeed = constrain(rightSpeed, -max_pwm, max_pwm);

      // Set PWM values for motor drivers
    ledcWrite(channelL, abs(leftSpeed));
    ledcWrite(channelR, abs(rightSpeed));
    
    // Set direction pins for left side
    if (leftSpeed >= 0) {
        digitalWrite(lmddir1, HIGH);
        digitalWrite(lmddir2, HIGH);
    } else {
        digitalWrite(lmddir1, LOW);
        digitalWrite(lmddir2, LOW);
    }
    
    // Set direction pins for right side
    if (rightSpeed >= 0) {
        digitalWrite(rmddir1, LOW);
        digitalWrite(rmddir2, LOW);
    } else {
        digitalWrite(rmddir1, HIGH);
        digitalWrite(rmddir2, HIGH);
    }
    
    Serial.printf("Left: %d, Right: %d\n", leftSpeed, rightSpeed);


}


// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    
    vTaskDelay(1) ;
}
