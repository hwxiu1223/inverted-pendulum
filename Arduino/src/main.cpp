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
  M - set motor speed, sent by Python
  R - request encoder data, sent by Python
  E - encoder data, sent by Arduino
  Examples:
  Set motor speed to 150:   M,150\n
  Request encoder data:     R\n
  Send encoder data:        E,4095,1024\n

Pins Setup:
  AS5600: VCC-power (5v or 3v3), GND-ground, SCL-SCL, SDA-SDA
  Motor Driver: VCC- (5v), GND-ground, R_EN-power, L_EN-power, R_PWM-digital 5, L_PWM-digital 6
*/

#include <Arduino.h>
#include <Wire.h>

#include "BTS7960.h"
#include "AS5600.h"

////// PIN ASSIGNMENTS //////
const uint8_t PIN_A = 2; 
const uint8_t PIN_B = 3;
const uint8_t R_PWM = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const uint8_t L_PWM = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
const uint8_t L_EN = 10;
const uint8_t R_EN = 9;

volatile long enc_count = 0;
volatile uint8_t enc_prevState = 0;

// index = (prevState<<2)|newState, each state is (A<<1)|B
// 0 = invalid/no-motion transition, +1/-1 = valid step
static const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void encoderISR() {
  digitalWrite(LED_BUILTIN, HIGH);
  uint8_t a = digitalRead(PIN_A);
  uint8_t b = digitalRead(PIN_B);
  uint8_t newState = (a << 1) | b;
  uint8_t index = (enc_prevState << 2) | newState;
  enc_count += QEM[index];
  enc_prevState = newState;
  digitalWrite(LED_BUILTIN, LOW);    
}

// Library Objects
BTS7960 motorDriver(L_EN, R_EN, L_PWM, R_PWM);
AS5600 magneticEncoder;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  motorDriver.Enable();
  magneticEncoder.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  
  uint_8_t a = digitalRead(PIN_A); //added this to fix lag
  uint_8_t b = digitalRead(PIN_B);
  enc_prevState = (a << 1) | b;

  attachInterrupt(digitalPinToInterrupt(PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encoderISR, CHANGE);

  while (!magneticEncoder.isConnected()) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);                  
    digitalWrite(LED_BUILTIN, LOW);    
    delay(100);  
  }

  char ready = 0;

  while (!ready) {
    Serial.println("REQ");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);                  
    digitalWrite(LED_BUILTIN, LOW);  
    delay(400);

    if (Serial.available() > 0) {
      String message = Serial.readStringUntil('\n');
      message.trim();

      if (message.startsWith("RET")) {
        int commaIndex = message.indexOf(',');
        String valueStr = message.substring(commaIndex + 1);
        enc_count = valueStr.toInt();

        Serial.println("REC");
        ready = 1;
      }
    }
  }
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
    else if (message == "R") {
      int magAngle = magneticEncoder.readAngle();
      
      // Send back: E,magAngle,rotaryCount\n
      Serial.print("E,");
      Serial.print(magAngle);
      Serial.print(",");
      Serial.println(enc_count);
    }

    // Reset arduino
    else if (message == "ST") {
      enc_count = 0;
      NVIC_SystemReset();
    }
  }          
  // Serial.println(enc_count);
}