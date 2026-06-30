/*
The purpose of the arduino is to be a homemade motor controller,
so its functions are:
- Drives motor
- Reads from motor encoders

vendor documentation:
- Motor Driver: https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf
- Motor: 
- Rotary Encoder:
- Magnetic Encoder:
*/


#include <Arduino.h>

#include "main.h"

int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // blinky
  digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on
  delay(500);                       // Wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // Turn the LED off
  delay(500);                       // Wait for a second
}

void drive_motor(int valuePWM, bool direction) {
  if (direction == 1) {
    analogWrite(LPWM_Output, valuePWM);
    analogWrite(RPWM_Output, 0);
  }
  else {
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, valuePWM);
  }
}