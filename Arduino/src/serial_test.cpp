#include <Arduino.h>

void setup() {
  Serial.begin(9600);           // Start serial communication at 9600 baud rate
  pinMode(LED_BUILTIN, OUTPUT); // Set the built-in LED as an output
}

void loop() {
  if (Serial.available() > 0) { // Check if data is sent from Python
    char data = Serial.read();  // Read the incoming byte
    
    if (data == '1') {
      digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
      Serial.println("LED turned ON"); // Send response back to Python
    } else if (data == '0') {
      digitalWrite(LED_BUILTIN, LOW);  // Turn LED off
      Serial.println("LED turned OFF");
    }
  }
}