/*
The purpose of the arduino is to be a homemade motor controller,
so its functions are:
- Drives motor
- Reads from rotary encoder and magnetic encoder

vendor documentation:
- Motor Driver: https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf
- Motor: https://manuals.plus/asin/B08FBD7BZW
- Rotary Encoder: none that ive found
- Magnetic Encoder: https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf

Libraries:
- Motor Driver (BTS7960): https://github.com/luisllamasbinaburo/Arduino-BTS7960
- Magnetic Encoder (AS5600): https://github.com/RobTillaart/AS5600/blob/master/README.md

Serial Communication Protocol:
  Command/Msg type, data, newline character
  Commands: 
  M - set motor speed
  R - request encoder data
  E - encoder data
  Examples:
  Set motor speed to 150:   M,150\n
  Request encoder data:     R\n
  Send encoder data:        E,4095,1024\n

Pins Setup:
  AS5600: VCC-power (5v or 3v3), GND-ground, SCL-SCL, SDA-SDA
*/

#include <Arduino.h>
#include <Wire.h>

#include "BTS7960.h"
#include "AS5600.h"

////// PIN ASSIGNMENTS //////
const uint8_t R_PWM = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const uint8_t L_PWM = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
const uint8_t L_EN = 4; // Left Enable pin (can be any digital pin)
const uint8_t R_EN = 3; // Right Enable pin (can be any digital pin)


// Library Objects
BTS7960 motorDriver(L_EN, R_EN, L_PWM, R_PWM);
AS5600 magneticEncoder;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  motorDriver.Enable();
  // magneticEncoder.begin();


//   while (!magneticEncoder.isConnected()) {
//     Serial.println("AS5600 not detected!");

//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(100);                  
//     digitalWrite(LED_BUILTIN, LOW);    
//     delay(100);  
//   }
}

void loop() {
  // Check if Python sent a message
  if (Serial.available() > 0) {
    // Read until the newline character
    String message = Serial.readStringUntil('\n');
    message.trim(); // Remove any leftover \r whitespace
    
    // Parse Motor Command (e.g., "M,150")
    if (message.startsWith("M")) {
      int commaIndex = message.indexOf(',');
      String speedString = message.substring(commaIndex + 1);
      int speed = speedString.toInt();
      
      if (speed >= 0) {
        motorDriver.TurnRight(speed);
      } 
      else {
        speed = -speed;
        motorDriver.TurnLeft(speed);
      }
    }
    
    // Parse Request for Data (e.g., "R")
    // else if (message == "R") {
    //   int magAngle = magneticEncoder.readAngle();
    //   int rotaryCount = 0; // Replace with your rotary encoder variable
      
    //   // Send back: E,magAngle,rotaryCount\n
    //   Serial.print("E,");
    //   Serial.print(magAngle);
    //   Serial.print(",");
    //   Serial.println(rotaryCount);
    // }
  }          
}