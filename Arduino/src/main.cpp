/*
The purpose of the arduino is to be a homemade motor controller,
so its functions are:
- Drives motor
- Reads from motor encoders

vendor documentation:
- Motor Driver: https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf
- Motor: https://manuals.plus/asin/B08FBD7BZW
- Rotary Encoder: none that ive found
- Magnetic Encoder: https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf

Libraries:
- Motor Driver (BTS7960): https://github.com/luisllamasbinaburo/Arduino-BTS7960
- Magnetic Encoder (AS5600): https://github.com/RobTillaart/AS5600/blob/master/README.md
*/


#include <Arduino.h>

#include "BTS7960.h"
#include "AS5600.h"

#include "main.h"

// PINS
const uint8_t R_PWM = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const uint8_t L_PWM = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
const uint8_t L_EN = 4; // Left Enable pin (can be any digital pin)
const uint8_t R_EN = 3; // Right Enable pin (can be any digital pin)


// Library Objects
BTS7960 motorDriver(L_EN, R_EN, L_PWM, R_PWM);
AS5600 rotaryEncoder;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  motorDriver.Enable();
  rotaryEncoder.begin();
  while (!rotaryEncoder.isConnected()) {
    Serial.println("AS5600 not detected!");

    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);                  
    digitalWrite(LED_BUILTIN, LOW);    
    delay(10);  
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);                  
    digitalWrite(LED_BUILTIN, LOW);    
    delay(50);  
  }
}

void loop() {
  // blinky
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);                       
  digitalWrite(LED_BUILTIN, LOW);    
  delay(500);                     
}