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

NOTE (fixed): all Serial parsing now uses fixed-size char buffers instead of
the Arduino String class. String does heap alloc/free on every operation
(readStringUntil, trim, substring), which fragments the tiny SRAM heap over
a long-running loop() and eventually causes a silent lockup with no RX/TX
activity. Fixed buffers use zero heap allocation, so this can run indefinitely.
*/

#include <Arduino.h>

#include "BTS7960.h"
// #include "AS5600.h"

////// PIN ASSIGNMENTS //////
const uint8_t PIN_A = 2;
const uint8_t PIN_B = 3;
const uint8_t R_PWM = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const uint8_t L_PWM = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
const uint8_t PEND_A = 7; //pend angle enc
const uint8_t PEND_B = 8;
const uint8_t L_EN = 10;
const uint8_t R_EN = 9;

volatile long enc_count = 0;
volatile uint8_t enc_prevState = 0;
// volatile uint8_t magAngle = 0;

volatile long pend_count = 0;
volatile uint8_t prev_pendState = 0;


// index = (prevState<<2)|newState, each state is (A<<1)|B
// 0 = invalid/no-motion transition, +1/-1 = valid step
static const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void encoderISR() { 
  uint8_t a = digitalRead(PIN_A);
  uint8_t b = digitalRead(PIN_B);
  uint8_t newState = (a << 1) | b;
  uint8_t index = (enc_prevState << 2) | newState;
  enc_count += QEM[index];
  enc_prevState = newState;
}


void pendulumEncoder_poll() {

  uint8_t a = digitalRead(PEND_A);
  uint8_t b = digitalRead(PEND_B);

  uint8_t newState = (a << 1) | b;

  uint8_t index = (prev_pendState << 2) | newState;

  pend_count += QEM[index];

  prev_pendState = newState;
}

// Library Objects
BTS7960 motorDriver(L_EN, R_EN, L_PWM, R_PWM);
// AS5600 magneticEncoder;

////// SERIAL PARSING BUFFER //////
// Fixed-size buffer replaces Arduino String to avoid heap fragmentation.
// 32 bytes is generous for "M,-255\n" / "R\n" / "ST\n" / "RET,65535\n" style messages;
// increase if you ever send longer commands.
const uint8_t BUF_SIZE = 32;
char inputBuffer[BUF_SIZE];

// Reads a line (up to '\n') into inputBuffer, trims trailing \r, null-terminates.
// Returns true if a full line was read, false if nothing was available.
bool readLineNonBlocking() {
  if (Serial.available() == 0) return false;

  size_t len = Serial.readBytesUntil('\n', inputBuffer, BUF_SIZE - 1);
  inputBuffer[len] = '\0';

  // Trim a trailing '\r' left over from CRLF line endings
  if (len > 0 && inputBuffer[len - 1] == '\r') {
    inputBuffer[len - 1] = '\0';
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.begin(115200);
delay(1000);
Serial.print("PIN_A interrupt#: ");  Serial.println(digitalPinToInterrupt(PIN_A));
Serial.print("PIN_B interrupt#: ");  Serial.println(digitalPinToInterrupt(PIN_B));
Serial.print("PEND_A interrupt#: "); Serial.println(digitalPinToInterrupt(PEND_A));
Serial.print("PEND_B interrupt#: "); Serial.println(digitalPinToInterrupt(PEND_B));
  // Wire.begin();
  // Wire.setClock(100000);       // stick to standard 100kHz — more noise-tolerant than 400kHz
  // Wire.setTimeout(50);         // ms; abort a stuck transaction instead of hanging indefinitely
  motorDriver.Enable();
  // magneticEncoder.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  pinMode(PEND_A, INPUT_PULLUP);
  pinMode(PEND_B, INPUT_PULLUP);
  uint8_t pendA = digitalRead(PEND_A);
  uint8_t pendB = digitalRead(PEND_B);
  prev_pendState = (pendA << 1) | pendB;


  attachInterrupt(digitalPinToInterrupt(PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encoderISR, CHANGE);
  

  // while (!magneticEncoder.isConnected()) {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(100);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(900);
  // }

  bool ready = false;

  while (!ready) {
    Serial.println("REQ");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);

    if (readLineNonBlocking()) {
      // Expecting "RET,<value>"
      if (strncmp(inputBuffer, "RET", 3) == 0) {
        char* commaPos = strchr(inputBuffer, ',');
        if (commaPos != nullptr) {
          enc_count = atol(commaPos + 1);
        }
        Serial.println("REC");
        ready = true;
      }
    }
  }
}

void loop() {
  pendulumEncoder_poll();
  if (readLineNonBlocking()) {
    // Parse Motor Command (e.g., "M,150")
    if (inputBuffer[0] == 'M') {
      char* commaPos = strchr(inputBuffer, ',');
      if (commaPos != nullptr) {
        int speed = atoi(commaPos + 1);

        if (speed >= 0) {
          motorDriver.TurnRight(speed);
        } else {
          motorDriver.TurnLeft(-speed);
        }
      }
    }

    // Parse Request for Data (e.g., "R")
    // else if (inputBuffer[0] == 'R' && inputBuffer[1] == '\0') {
    //   magAngle = magneticEncoder.readAngle();

    //   // Send back: E,magAngle,rotaryCount\n
    //   Serial.print("E,");
    //   Serial.print(magAngle);
    //   Serial.print(",");
    //   Serial.println(enc_count);
    // }
  else if (inputBuffer[0] == 'R' && inputBuffer[1] == '\0') {

  long currentPendCount;
  long currentCartCount;

  noInterrupts();

  currentPendCount = pend_count;
  currentCartCount = enc_count;

  interrupts();

  // Send back:
  // E,pendulumCount,cartCount

  Serial.print("E,");
  Serial.print(currentPendCount);
  Serial.print(",");
  Serial.println(currentCartCount);
  }

    // Reset arduino
    else if (strcmp(inputBuffer, "ST") == 0) {
      enc_count = 0;
      NVIC_SystemReset();
    }
  }
}