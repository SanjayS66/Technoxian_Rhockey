#include <Bluepad32.h>

// Array to store controller pointers
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Called when a controller is connected or disconnected
void onControllerConnected(ControllerPtr ctl) {
  Serial.println("Controller connected");
  for (int i = 0; i < BP32_MAX_CONTROLLERS; ++i) {
    if (!myControllers[i]) {
      myControllers[i] = ctl;
      return;
    }
  }
}

void onControllerDisconnected(ControllerPtr ctl) {
  Serial.println("Controller disconnected");
  for (int i = 0; i < BP32_MAX_CONTROLLERS; ++i) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ; // Wait for serial console

  BP32.setup(&onControllerConnected, &onControllerDisconnected);
  BP32.enableNewBluetoothConnections(true); // Accept new controllers
}

void loop() {
  // Processes BT events and updates controllers
  BP32.update();

  // Iterate through connected controllers
  for (int i = 0; i < BP32_MAX_CONTROLLERS; ++i) {
    ControllerPtr ctl = myControllers[i];
    if (ctl && ctl->isConnected()) {
      // Example: Print button state
      Serial.print("A: ");
      Serial.print(ctl->a());
      Serial.print("  B: ");
      Serial.print(ctl->b());
      Serial.print("  X: ");
      Serial.print(ctl->x());
      Serial.print("  Y: ");
      Serial.println(ctl->y());

      // Example: Print analog stick values
      Serial.print("Left stick X: ");
      Serial.print(ctl->axisX());
      Serial.print("  Y: ");
      Serial.println(ctl->axisY());

      Serial.print("Right stick X: ");
      Serial.print(ctl->axisRX());
      Serial.print("  Y: ");
      Serial.println(ctl->axisRY());

      // Example: Print triggers
      Serial.print("LT: ");
      Serial.print("  RT: ");


      Serial.println("---");
      delay(200); // Adjust refresh as needed
    }
  }
}
