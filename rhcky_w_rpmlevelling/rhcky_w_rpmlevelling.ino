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
int baseSpeed       = 90;   // default cruising speed with just joystick input
int currentSpeed    = baseSpeed;
int speedChangeRate = 8;    // ramp up/down step
int joystickDeadzone = 20;  // sticks under this are “zero”

// —————— RPM CALIBRATION ——————(USE ONLY IF BOTH MOTOTRS HAVE DIFFERENT RPM)
// Only scale the faster (224 RPM) motor down to 160 RPM.
// Leave the 168 RPM motor unscaled.
const float LEFT_MOTOR_SCALE  = 1.0f;           // 168 RPM motor stays 0–168
const float RIGHT_MOTOR_SCALE = 160.0f / 224.0f;  // caps 224 RPM → 160 RPM

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

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
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
    int yAxis    = map(ctl->axisY(), -511, 512, -255, 255);
    int xAxis    = map(ctl->axisX(), -511, 512, -255, 255);
    int accel    = ctl->throttle();  // 0–1023
    int brake    = ctl->brake();     // 0–1023
    bool pturn   = ctl->buttons() & 0x0004;

    if (pturn) {
        // point turn
        int target = min(maxSpeed, currentSpeed + speedChangeRate);
        currentSpeed += (currentSpeed < target) ? speedChangeRate : -speedChangeRate;
        setMotorSpeeds( currentSpeed, -currentSpeed );
    } else {
        // deadzones
        if (abs(yAxis) < joystickDeadzone) yAxis = 0;
        if (abs(xAxis) < joystickDeadzone) xAxis = 0;

        // throttle/brake overrides
        if (accel > 15) {
            int tgt = map(accel, 0, 1023, baseSpeed, maxSpeed);
            currentSpeed = min(currentSpeed + speedChangeRate, tgt);
        }
        else if (brake > 15) {
            int brk = map(brake, 0, 1023, 0, baseSpeed);
            int tgt = max(baseSpeed - brk, 0);
            currentSpeed = max(currentSpeed - speedChangeRate*2, tgt);
        }
        else {
            if (currentSpeed < baseSpeed) currentSpeed += speedChangeRate;
            else if (currentSpeed > baseSpeed) currentSpeed -= speedChangeRate;
        }

        // differential drive mix
        float scale = (float)currentSpeed / maxSpeed;
        int forward = (int)(yAxis * scale);
        int leftSp  = constrain(forward + xAxis, -max_pwm, max_pwm);
        int rightSp = constrain(forward - xAxis, -max_pwm, max_pwm);
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
                  bd[0],bd[1],bd[2],bd[3],bd[4],bd[5]);

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
    // apply scaling
    int pwmL = constrain(int(abs(leftSpeed)  * LEFT_MOTOR_SCALE),  0, max_pwm);
    int pwmR = constrain(int(abs(rightSpeed) * RIGHT_MOTOR_SCALE), 0, max_pwm);

    ledcWrite(channelL, pwmL);
    ledcWrite(channelR, pwmR);

    digitalWrite(lmddir, leftSpeed  >= 0 ? HIGH : LOW);
    digitalWrite(rmddir, rightSpeed >= 0 ? HIGH : LOW);
}

void loop() {
    if (BP32.update()) {
        processControllers();
    }
    vTaskDelay(1);
}