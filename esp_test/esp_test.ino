// Define the pin where the LED is connected
const int ledPin = 2; // On-board LED is usually on GPIO 2

void setup() {
  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH); // Turn the LED on
  delay(1000);                // Wait for 1 second
  digitalWrite(ledPin, LOW);  // Turn the LED off
  delay(1000);
  digitalWrite(ledPin, LOW);  // Turn the LED off
  delay(1000);                // Wait for 1 second
  digitalWrite(ledPin, LOW);  // Turn the LED off
  delay(1000);                // Wait for 1 second
  digitalWrite(ledPin, LOW);  // Turn the LED off
  delay(1000);                // Wait for 1 second
  digitalWrite(ledPin, LOW);  // Turn the LED off
  delay(1000);    
  Serial.print("asgfdjgndfjnbjdfnbjnjngb")  ;          // Wait for 1 second                // Wait for 1 second
}

