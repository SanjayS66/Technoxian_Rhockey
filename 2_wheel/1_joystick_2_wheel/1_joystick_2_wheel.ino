#include <Bluepad32.h>

// —————— PIN DEFINITIONS ——————
int rmdpwm = 33;   // Right motor PWM
int rmddir = 32;   // Right motor DIR
int lmdpwm = 13;   // Left  motor PWM
int lmddir = 14;   // Left  motor DIR

// —————— PWM CONFIG ——————
int freq       = 5000;  // PWM frequency
int res        = 8;     // 8‑bit resolution (0–255)
int channelR   = 0;     // right PWM channel
int channelL   = 1;     // left  PWM channel
int max_pwm    = 255;   // absolute PWM limit

// —————— SPEED CONTROL ——————
int maxSpeed        = 255;  // maps to “full throttle”
int baseSpeed       = 90;   // default cruising speed
int currentSpeed    = baseSpeed;
int speedChangeRate = 4;    // ramp up/down step
int joystickDeadzone = 30;  // smaller deadzone for more control
float turnSensitivity = 0.6;




// —————— BLUPAD32 GAMEPAD ——————
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            return;
        }
    }
    Serial.println("CALLBACK: No empty slot for new controller");
}

// Function declaration
void setMotorSpeeds(int leftSpeed, int rightSpeed);


void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            setMotorSpeeds(0,0);
            return;
        }
    }
}


// Debug print
void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, axis L: %4d, %4d, throttle: %4d, brake: %4d\n",
        ctl->index(), ctl->axisX(), ctl->axisY(),
        ctl->throttle(), ctl->brake()
    );
}

void processGamepad(ControllerPtr ctl) {
    float yAxis = (float)(ctl->axisY());
    float xAxis = (float)(ctl->axisX());

    int accel = ctl->throttle();  // 0–1023
    int brake = ctl->brake();     // 0–1023
    bool pturn = ctl->buttons() & 0x0004;
bool fakeShot = ctl->buttons() & 0x0008;  // Traingle button for fakeshot(ended up being emote :()


    // Emergency stop check (example: both shoulder buttons)
    if ((ctl->buttons() & 0x0030) == 0x0030) {  // L1 + R1 pressed
        setMotorSpeeds(0, 0);
        Serial.println("EMERGENCY STOP ACTIVATED");
        return;
    }


    if (pturn) {
        // point turn
        int target = min(maxSpeed / 2, currentSpeed + speedChangeRate);
        currentSpeed += (currentSpeed < target) ? speedChangeRate : -speedChangeRate;
        setMotorSpeeds(currentSpeed, -currentSpeed);
    } else if (fakeShot) { //emote 
        setMotorSpeeds(200, 200);   // Quick forward
        delay(200);
        setMotorSpeeds(0, 0);       // Sudden stop
        delay(50);
        setMotorSpeeds(-100, 150);  // Quick turn
        delay(350);
        setMotorSpeeds(100,-150) ;   // back to previous turn
        delay(350);
        setMotorSpeeds(-100, 150);  // Quick turn
        delay(350);
        setMotorSpeeds(100,-150) ;   // back to previous turn
        delay(350);
        setMotorSpeeds(0,0);
        delay(50);
        setMotorSpeeds(200, 200);   // Continue forward
    } else {
        // deadzones
        if (abs(yAxis) < joystickDeadzone) yAxis = 0;
        if (abs(xAxis) < joystickDeadzone) xAxis = 0;


        //*test part*
        // Normalize to -1.0 to +1.0 range (adjust 512 based on your controller's range)
        yAxis = yAxis / 512.0;
        xAxis = xAxis / 512.0;
        
        // Clamp to prevent overflow
        yAxis = constrain(yAxis, -1.0, 1.0);
        xAxis = constrain(xAxis, -1.0, 1.0);

        // Apply turn sensitivity reduction
        xAxis *= turnSensitivity;

        // throttle/brake overrides
        if (accel > 25) {
            int tgt = map(accel, 0, 1023, baseSpeed, maxSpeed);
            currentSpeed = min(currentSpeed + speedChangeRate, tgt);
        } else if (brake > 25) {
            int brk = map(brake, 0, 1023, 0, baseSpeed);
            int tgt = max(baseSpeed - brk, 0);
            currentSpeed = max(currentSpeed - speedChangeRate * 2, tgt);
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
        setMotorSpeeds(leftSp, rightSp);
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
    int pwmL = constrain(abs(leftSpeed), 0, max_pwm);
    int pwmR = constrain(abs(rightSpeed), 0, max_pwm);

    ledcWrite(channelL, pwmL);
    ledcWrite(channelR, pwmR);

    digitalWrite(lmddir, leftSpeed >= 0 ? LOW : HIGH);
    digitalWrite(rmddir, rightSpeed >= 0 ? LOW : HIGH);
}

void loop() {
    if (BP32.update()) {
        processControllers();
    }
    vTaskDelay(1);
}

