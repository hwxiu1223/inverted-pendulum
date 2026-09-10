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

NOTE (rewired): the PENDULUM encoder is now the one wired to pins 2/3 and is
interrupt-driven (attachInterrupt), because on the UNO R4 only pins 2/3 are
pin-change-interrupt-capable. The CART encoder is now wired to pins 7/8 and
is instead polled from a hardware timer (FspTimer) at a fixed rate, since
7/8 can't be used with attachInterrupt() on this board. This is the mirror
image of the previous wiring/logic (cart was interrupt-driven, pendulum was
timer-polled) - the variable names (enc_count = cart, pend_count = pendulum)
and the serial protocol are unchanged.
*/

#include <Arduino.h>
#include "FspTimer.h"

#include "BTS7960.h"
// #include "AS5600.h"

////// PIN ASSIGNMENTS //////
const uint8_t PEND_A = 2; // pendulum angle encoder - interrupt-capable pin
const uint8_t PEND_B = 3; // pendulum angle encoder - interrupt-capable pin
const uint8_t R_PWM = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const uint8_t L_PWM = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
const uint8_t PIN_A = 7; // cart encoder - polled via timer, not interrupt-capable on this board
const uint8_t PIN_B = 8; // cart encoder - polled via timer, not interrupt-capable on this board
const uint8_t L_EN = 10;
const uint8_t R_EN = 9;

volatile long enc_count = 0;          // cart encoder count (polled)
volatile uint8_t enc_prevState = 0;   // cart encoder previous quadrature state
// volatile uint8_t magAngle = 0;

volatile long pend_count = 0;             // pendulum encoder count (interrupt-driven)
volatile uint8_t prev_pendState = 0;      // pendulum encoder previous quadrature state


// index = (prevState<<2)|newState, each state is (A<<1)|B
// 0 = invalid/no-motion transition, +1/-1 = valid step
static const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// Interrupt-driven: fires on any CHANGE of PEND_A/PEND_B (pins 2/3).
void pendulumEncoderISR() {
  uint8_t a = digitalRead(PEND_A);
  uint8_t b = digitalRead(PEND_B);
  uint8_t newState = (a << 1) | b;
  uint8_t index = (prev_pendState << 2) | newState;
  pend_count += QEM[index];
  prev_pendState = newState;
}


// Called from a hardware timer interrupt (see beginCartTimer), NOT from loop().
// This guarantees a fixed poll rate regardless of what loop() is doing (serial
// parsing, Serial.print, motor commands, etc.), so we don't miss quadrature
// transitions during a blocking Serial call.
void cartEncoder_poll(timer_callback_args_t *arg) {
  uint8_t a = digitalRead(PIN_A);
  uint8_t b = digitalRead(PIN_B);

  uint8_t newState = (a << 1) | b;

  uint8_t index = (enc_prevState << 2) | newState;

  enc_count += QEM[index];

  enc_prevState = newState;
}

// Sets up a Renesas GPT timer to call cartEncoder_poll() at a fixed rate.
// Needed because PIN_A/PIN_B (pins 7/8) are not attachInterrupt()-capable
// on the UNO R4 (only pins 2/3 are), so we can't use pin-change interrupts
// for this encoder like we do for the pendulum encoder.
FspTimer cartTimer;

bool beginCartTimer(float rateHz) {
  uint8_t timerType = GPT_TIMER;
  int8_t tIndex = FspTimer::get_available_timer(timerType);
  if (tIndex < 0) tIndex = FspTimer::get_available_timer(timerType, true);
  if (tIndex < 0) return false;

  FspTimer::force_use_of_pwm_reserved_timer();

  if (!cartTimer.begin(TIMER_MODE_PERIODIC, timerType, tIndex, rateHz, 0.0f, cartEncoder_poll)) return false;
  // IMPORTANT: explicit low priority (15 = lowest on this core).
  // FspTimer::setup_overflow_irq() defaults to priority 12, which is the
  // SAME priority the R4's USB/Serial interrupt runs at. Equal-priority
  // interrupts can't preempt each other, so a 10kHz timer at priority 12
  // repeatedly contends with USB servicing and eventually starves it,
  // causing exactly the "USB looks frozen" symptom. Priority 15 makes sure
  // USB always wins that contention.
  if (!cartTimer.setup_overflow_irq(15)) return false;
  if (!cartTimer.open()) return false;
  if (!cartTimer.start()) return false;
  return true;
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
  // Default Serial timeout is 1000ms: if readBytesUntil() is called with a
  // partial line already in the buffer (no '\n' yet), it will BLOCK for up
  // to this long waiting for more bytes. During that block, nothing else in
  // loop() runs. Keep this short so a partial/garbled line can't stall us.
  Serial.setTimeout(5);

  delay(100);
  Serial.print("PEND_A interrupt#: "); Serial.println(digitalPinToInterrupt(PEND_A));
  Serial.print("PEND_B interrupt#: "); Serial.println(digitalPinToInterrupt(PEND_B));
  Serial.print("PIN_A interrupt#: ");  Serial.println(digitalPinToInterrupt(PIN_A));
  Serial.print("PIN_B interrupt#: ");  Serial.println(digitalPinToInterrupt(PIN_B));
  // Wire.begin();
  // Wire.setClock(100000);       // stick to standard 100kHz — more noise-tolerant than 400kHz
  // Wire.setTimeout(50);         // ms; abort a stuck transaction instead of hanging indefinitely
  motorDriver.Enable();
  // magneticEncoder.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PEND_A, INPUT_PULLUP);
  pinMode(PEND_B, INPUT_PULLUP);
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);

  // Initialize the pendulum's prev-state before attaching its interrupt so
  // the first transition after boot is computed correctly.
  uint8_t pendA = digitalRead(PEND_A);
  uint8_t pendB = digitalRead(PEND_B);
  prev_pendState = (pendA << 1) | pendB;

  // Initialize the cart's prev-state before the timer starts polling it,
  // for the same reason.
  uint8_t cartA = digitalRead(PIN_A);
  uint8_t cartB = digitalRead(PIN_B);
  enc_prevState = (cartA << 1) | cartB;

  attachInterrupt(digitalPinToInterrupt(PEND_A), pendulumEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PEND_B), pendulumEncoderISR, CHANGE);

  if (!beginCartTimer(80000.0f)) {
    Serial.println("CART_TIMER_FAIL");
  }

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
  // TEMPORARY DIAGNOSTIC: blink LED_BUILTIN every 250ms using only millis(),
  // no Serial calls involved. If this keeps blinking during a "freeze", the
  // CPU is alive and the real problem is Serial.print() blocking (e.g. your
  // Python side not reading fast enough, filling the USB TX buffer). If this
  // ALSO stops blinking, the CPU/ISR side has actually locked up. Remove once
  // the freeze cause is confirmed.
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink >= 250) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
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

    else if (inputBuffer[0] == 'R' && inputBuffer[1] == '\0') {

      long currentPendCount;
      long currentCartCount;

      noInterrupts();

      currentPendCount = pend_count;
      currentCartCount = enc_count;

      interrupts();

      // Send back:
      // E,cartCount,pendulumCount

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
