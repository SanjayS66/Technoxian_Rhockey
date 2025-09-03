#include <Bluepad32.h>
#define LED_PIN 2

// —————— PIN DEFINITIONS ——————
int rmdpwm = 33;   // Right motor PWM
int rmddir =32;   // Right motor DIR
int lmdpwm = 26;   // Left  motor PWM
int lmddir = 25;   // Left  motor DIR

// —————— PWM CONFIG ——————
int freq       = 16000;  // PWM frequency
int res        = 8;     // 8‑bit resolution (0–255)
int channelR   = 0;     // right PWM channel
int channelL   = 1;     // left  PWM channel
int max_pwm    = 255;   // absolute PWM limit

// —————— SPEED CONTROL ——————
int maxSpeed        = 255;  // speed for “full throttle”
int baseSpeed       = 150;   // default cruising speed
int currentSpeed    = baseSpeed;   //variable to store calculated speed based on joystick+throttle input to be passed to set motor function
int speedChangeRate = 4;    // speed changes over every loop at this rate to prevent jerking
int joystickDeadzone = 70;  //to account for the error in the controller joystick
float turnSensitivity = 0.55;   //for better control. without this bot starts taking pturns for small x-input
                               //Change turn sensitivity for changing turning radius

// —————— BLUPAD32 GAMEPAD(nothing much to change) ——————
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
    for (int i = 0; i < 1; i++) {
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
        "idx=%d, axis L: %4d, %4d, throttle: %4d, brake: %4d, axis R: %4d, %4d, buttons: 0x%04X\n",
        ctl->index(), ctl->axisX(), ctl->axisY(),
        ctl->throttle(), ctl->brake(),ctl->axisRX(),ctl->axisRY(), ctl->buttons()
    );
}

//main differential drive logic
void processGamepad(ControllerPtr ctl) {
    float yAxis = (float)(ctl->axisY());       //joystick values
    float xAxis = (float)(ctl->axisRX());

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
        int target = min(int(maxSpeed), currentSpeed + speedChangeRate);
        currentSpeed += (currentSpeed < target) ? speedChangeRate : -speedChangeRate;
        setMotorSpeeds(currentSpeed, -currentSpeed);}
    else if(cwpturn){
        // cw point turn
        int target = min(int(maxSpeed), currentSpeed + speedChangeRate);
        currentSpeed += (currentSpeed < target) ? speedChangeRate : -speedChangeRate;
        setMotorSpeeds(-currentSpeed, currentSpeed);
    }
    // else if (fakeShot) { //emote 

    //     digitalWrite(LED_PIN, LOW);
    //     delay(20);  
    //     setMotorSpeeds(200, 200);        // Quick forward
    //     digitalWrite(LED_PIN, HIGH);  
    //     delay(200);
    //     setMotorSpeeds(0, 0);       // Sudden stop
    //     digitalWrite(LED_PIN, LOW);
    //     delay(50);
    //     digitalWrite(LED_PIN, HIGH); 
    //     setMotorSpeeds(-100, 150);  // Quick turn
    //     delay(350);
    //     digitalWrite(LED_PIN, LOW);
    //     setMotorSpeeds(100,-150) ;      // back to previous turn   
    //     delay(350);
    //     digitalWrite(LED_PIN, HIGH);
    //     setMotorSpeeds(-100, 150);  // Quick turn
    //     delay(350);
    //     digitalWrite(LED_PIN, LOW);
    //     setMotorSpeeds(100,-150) ;   // back to previous turn
    //     delay(350);
    //     digitalWrite(LED_PIN, HIGH);
    //     setMotorSpeeds(0,0);
    //     delay(50);
    //     digitalWrite(LED_PIN, LOW);
    //     setMotorSpeeds(200, 200);   // Continue forward
    // }
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
        
        int leftSp  = constrain(forward + turn, -max_pwm, max_pwm);
        int rightSp = constrain(forward - turn, -max_pwm, max_pwm);
        setMotorSpeeds(leftSp, rightSp*0.94);        //final minute fix to make the bot go straighter even though the placement of motor is the real issue
    }

    dumpGamepad(ctl);
}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            if (ctl->isGamepad()) processGamepad(ctl);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* bd = BP32.localBdAddress();
    Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  bd[0], bd[1], bd[2], bd[3], bd[4], bd[5]);

    //led blink with emote
    pinMode(LED_PIN, OUTPUT); 

    // PWM channels
    ledcSetup(channelL, freq, res);
    ledcSetup(channelR, freq, res);
    
    ledcAttachPin(lmdpwm, channelL);
    ledcAttachPin(rmdpwm, channelR);

    pinMode(lmddir, OUTPUT);
    pinMode(rmddir, OUTPUT);

    setMotorSpeeds(0, 0);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {




    // Constrain to absolute range
    leftSpeed = constrain(leftSpeed, -max_pwm, max_pwm);
    rightSpeed = constrain(rightSpeed, -max_pwm, max_pwm);
    // Convert 0–255 to 20–240 range (8%–94%) //for the smartelex 15d motor driver which accepts pwm in this range only for some reason   
    int pwmR = map(abs(rightSpeed), 0, 255, 0, 255);
    int pwmL = map(abs(leftSpeed), 0, 255, 0, 255);

    // Write PWM
    ledcWrite(channelL, (leftSpeed == 0) ? 0 : pwmL);
    ledcWrite(channelR, (rightSpeed == 0) ? 0 : pwmR);

    // Direction control
    digitalWrite(lmddir, leftSpeed >= 0 ? HIGH : LOW);
    digitalWrite(rmddir, rightSpeed >= 0 ? HIGH : LOW);

    Serial.printf("Left: %d, Right: %d\n", leftSpeed, rightSpeed);
}


void loop() {
    if (BP32.update()) {
        processControllers();
    }
    vTaskDelay(1);
}


