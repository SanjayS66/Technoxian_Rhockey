#include <Bluepad32.h>

// —————— PIN DEFINITIONS ——————
int rmdpwm1 = 17;    //Right motor 1 pwm
int rmdpwm2 = 4;     //Right motor 2 pwm
int rmddir1 = 5;     //Right motor 1 direction
int rmddir2 = 16;    //Right motor 2 direction
int lmdpwm1 = 21;    //Left motor 1 pwm
int lmdpwm2 = 1;     //Left motor 2 pwm
int lmddir1 = 19;    //Left motor 1 direction
int lmddir2 = 3;     //Left motor 2 direction
 

//PWM VARIABLES *if you are using ledc functio for custom pwm*
int freq = 5000;           // PWM frequency 
int res = 8;        // 8-bit resolution (0-255)

// PWM channel assignments(timer that helps generates pwm signal)
int channelR = 0;        // PWM channel for right motors
int channelL = 1;        // PWM channel for left motors
int max_pwm = 255;

//speed params
int maxSpeed = 255;     //speed for "full throttle"
int baseSpeed = 90;     //base cruising speed
int currentSpeed = baseSpeed;    //variable to store calculated speed based on joystick+throttle input to be passed to set motor function
int speedChangeRate = 4;    // speed changes over every loop at this rate to prevent jerking
int joystickDeadzone = 50;    //to account for the error in the controller joystick
float turnSensitivity = 0.4 ;   //for better control. without this bot starts taking pturns for small x-input

//******************CODE FROM EXAMPLE SKETCH(NOTHING TO CHANGE)*********************
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.but estricted to 1 controller at a time
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
// for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
     for (int i = 0; i < 1; i++) {                             //to restrict maximum connected controllers to 1
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;      //pointer to the controller
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}


// Function declaration so that it wont throw error on compilation
void setMotorSpeeds(int leftSpeed, int rightSpeed);

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            setMotorSpeeds(0,0);  //so bot doesnt keep going when controller gets disconnected
            return;
        }
    }
}

//Printing values on serial monitor for debuging

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x,\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons()  // bitmask of pressed "misc" buttons

    ); 
}

//main differential drive logic
void processGamepad(ControllerPtr ctl) {
    float yAxis = (float)(ctl->axisY());   //left joystick for forward & reverse
    float xAxis = (float)(ctl->axisRX());  //right joystick for left & right

    int accel = ctl->throttle();  // 0–1023
    int brake = ctl->brake();     // 0–1023
    bool ccwpturn = ctl->buttons() & 0x0004; //pturn in the CCW direction by ▢ (in PS4) and X (in XBox)
    bool cwpturn = ctl->buttons() & 0x0002; //pturn in the CW direction by O (in PS4) and B (in XBox)
    bool fakeShot = ctl->buttons() & 0x0008;  // Traingle button for fakeshot(ended up being emote :()


    // so that brake can overwrite accelerate value
    if (brake>150){
        accel = 0;
        Serial.printf("Brakes activated  brake : %d      accel : %d   ",brake,accel);    
    }

    // Emergency stop check (example: both shoulder buttons)
    if ((ctl->buttons() & 0x0030) == 0x0030) {  // L1 + R1 pressed
        setMotorSpeeds(0, 0);
        Serial.println("EMERGENCY STOP ACTIVATED");
        return;
    }


    if (ccwpturn) {
        // ccw point turn
        int target = min(int(maxSpeed * 0.6), currentSpeed + speedChangeRate);
        currentSpeed += (currentSpeed < target) ? speedChangeRate : -speedChangeRate;
        setMotorSpeeds(currentSpeed, -currentSpeed);}
    else if(cwpturn){
        // cw point turn
        int target = min(int(maxSpeed * 0.6), currentSpeed + speedChangeRate);
        currentSpeed += (currentSpeed < target) ? speedChangeRate : -speedChangeRate;
        setMotorSpeeds(-currentSpeed, currentSpeed);
    }
    else {
        // deadzones
        if (abs(yAxis) < joystickDeadzone) yAxis = 0;
        if (abs(xAxis) < joystickDeadzone) xAxis = 0;


        // Normalize to -1.0 to +1.0 range (adjust 512 based on your controller's range)
        yAxis = yAxis / 512.0;
        xAxis = xAxis / 512.0;
        
        // Clamp to prevent overflow
        yAxis = constrain(yAxis, -1.0, 1.0);
        xAxis = constrain(xAxis, -1.0, 1.0);

        // Apply turn sensitivity reduction
        xAxis *= turnSensitivity;

        // throttle/brake overrides
        if (accel > 50) {
            int tgt = map(accel, 0, 1023, baseSpeed, maxSpeed);
            currentSpeed = min(currentSpeed + speedChangeRate, tgt);
        } else if (brake > 50) {
            int brk = map(brake, 0, 1023, 0, baseSpeed);
            int tgt = max(baseSpeed - brk, 0);
            currentSpeed = max(int((currentSpeed - speedChangeRate )*1.6), tgt);
        } else {
            if (currentSpeed < baseSpeed) currentSpeed = min(currentSpeed + speedChangeRate, baseSpeed);
            else if (currentSpeed > baseSpeed) currentSpeed = max(currentSpeed - speedChangeRate, baseSpeed);
        }

        //test part ends

        // differential drive mix with proper scaling
        float scale = (float)currentSpeed / (float)maxSpeed;
        int forward = (int)(yAxis * scale * maxSpeed);
        int turn = (int)(xAxis * scale * maxSpeed);
        
        int leftSp  = constrain(forward - turn, -max_pwm, max_pwm);
        int rightSp = constrain(forward + turn, -max_pwm, max_pwm);
        Serial.printf("LEFT MOTOR SPEED : %d      RIGHT MOTOR SPEED : %d         ",leftSp,rightSp);
        setMotorSpeeds(leftSp, rightSp);
    }

    dumpGamepad(ctl);
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
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
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}


void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  
  int pwmL = constrain(abs(leftSpeed), 0, max_pwm);
  int pwmR = constrain(abs(rightSpeed), 0, max_pwm);

    
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
        digitalWrite(rmddir1, HIGH);
        digitalWrite(rmddir2, HIGH);
    } else {
        digitalWrite(rmddir1, LOW);
        digitalWrite(rmddir2, LOW);
    }
    

    // Write PWM
    ledcWrite(channelL, (leftSpeed == 0) ? 0 : pwmL);
    ledcWrite(channelR, (rightSpeed == 0) ? 0 : pwmR);

    // // Direction control
    // digitalWrite(lmddir, leftSpeed >= 0 ? LOW : HIGH);
    // digitalWrite(rmddir, rightSpeed >= 0 ? LOW : HIGH);
    Serial.printf("Left: %d, Right: %d\n", leftSpeed, rightSpeed);


}


// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    vTaskDelay(1) ;
}
