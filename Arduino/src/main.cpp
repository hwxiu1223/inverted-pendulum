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
  Send encoder data:        E,<pendulumCount>,<cartCount>\n

Pins Setup:
  AS5600: VCC-power (5v or 3v3), GND-ground, SCL-SCL, SDA-SDA
  Motor Driver: VCC- (5v), GND-ground, R_EN-power, L_EN-power, R_PWM-digital 5, L_PWM-digital 6
  Pendulum encoder: A-D2, B-D3
  Cart encoder:     A-D12, B-D8   (REWIRED - A used to be on D7)

NOTE (fixed): all Serial parsing now uses fixed-size char buffers instead of
the Arduino String class. String does heap alloc/free on every operation
(readStringUntil, trim, substring), which fragments the tiny SRAM heap over
a long-running loop() and eventually causes a silent lockup with no RX/TX
activity. Fixed buffers use zero heap allocation, so this can run indefinitely.

The cart encoder is now interrupt-driven, using the RA4M1's ICU (external IRQ
controller) directly through FSP instead of attachInterrupt():
  - D12 = P110 = IRQ3  (cart A)
  - D8  = P304 = IRQ9  (cart B)
D7 (P107) has no IRQ function in silicon, so it can't be used. The pendulum
still uses attachInterrupt() on D2 (IRQ0) / D3 (IRQ1). Every encoder pin is on
its own IRQ channel. Variable names (enc_count = cart, pend_count = pendulum)
and the serial protocol are unchanged.
*/

#include <Arduino.h>
#include "IRQManager.h"
#include "r_icu.h"

#include "BTS7960.h"
// #include "AS5600.h"

////// PIN ASSIGNMENTS //////
const uint8_t PEND_A = 2; // pendulum angle encoder - attachInterrupt (IRQ0)
const uint8_t PEND_B = 3; // pendulum angle encoder - attachInterrupt (IRQ1)
const uint8_t R_PWM = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const uint8_t L_PWM = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
const uint8_t L_EN = 10;
const uint8_t R_EN = 9;

// ICU external interrupts
const bsp_io_port_pin_t CART_A_PIN = BSP_IO_PORT_04_PIN_10;  // header D12
const bsp_io_port_pin_t CART_B_PIN = BSP_IO_PORT_03_PIN_04;  // header D8
const uint8_t CART_A_IRQ = 5;   // P110 -> IRQ3
const uint8_t CART_B_IRQ = 9;   // P304 -> IRQ9

volatile long enc_count = 0;          // cart encoder count (interrupt-driven, ICU)
volatile uint8_t enc_prevState = 0;   // cart encoder previous quadrature state
// volatile uint8_t magAngle = 0;

volatile long pend_count = 0;             // pendulum encoder count (interrupt-driven)
volatile uint8_t prev_pendState = 0;      // pendulum encoder previous quadrature state

volatile uint32_t cartIsrHits = 0;  // add near your other volatiles


// index = (prevState<<2)|newState, each state is (A<<1)|B
// 0 = invalid/no-motion transition, +1/-1 = valid step
static const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ISR for rotary encoder
void pendulumEncoderISR() {
  uint8_t a = digitalRead(PEND_A);
  uint8_t b = digitalRead(PEND_B);
  uint8_t newState = (a << 1) | b;
  uint8_t index = (prev_pendState << 2) | newState;
  pend_count += QEM[index];
  prev_pendState = newState;
}

// ICU control/config blocks
static icu_instance_ctrl_t cartA_ctrl, cartB_ctrl;
static external_irq_cfg_t  cartA_cfg,  cartB_cfg;

// Configures one pin as an external IRQ input on the given ICU channel,
// bypassing attachInterrupt().
bool setupExternalIrq(icu_instance_ctrl_t &ctrl, external_irq_cfg_t &cfg,
                      bsp_io_port_pin_t pin, uint8_t channel,
                      void (*cb)(external_irq_callback_args_t *)) {
  // Route the pin to the ICU: input, internal pullup, IRQ input enabled (ISEL bit)
  if (R_IOPORT_PinCfg(&g_ioport_ctrl, pin,
        IOPORT_CFG_PORT_DIRECTION_INPUT |
        IOPORT_CFG_PULLUP_ENABLE |
        IOPORT_CFG_IRQ_ENABLE) != FSP_SUCCESS) return false;

  cfg.irq           = FSP_INVALID_VECTOR;
  cfg.channel       = channel;
  cfg.trigger       = EXTERNAL_IRQ_TRIG_BOTH_EDGE;  // quadrature needs both edges
  cfg.filter_enable = false;                        // digital filter adds latency
  cfg.pclk_div      = EXTERNAL_IRQ_PCLK_DIV_BY_1;
  cfg.ipl           = 15;
  cfg.p_callback    = cb;
  cfg.p_context     = nullptr;
  cfg.p_extend      = nullptr;

  if (!IRQManager::getInstance().addPeripheral(IRQ_EXTERNAL_PIN, &cfg)) return false;

  if (R_ICU_ExternalIrqOpen(&ctrl, &cfg) != FSP_SUCCESS) return false;
  if (R_ICU_ExternalIrqEnable(&ctrl) != FSP_SUCCESS) return false;
  return true;
}

// Library Objects
BTS7960 motorDriver(L_EN, R_EN, L_PWM, R_PWM);
// AS5600 magneticEncoder;

const uint8_t BUF_SIZE = 32;
char inputBuffer[BUF_SIZE];

bool readLineNonBlocking() {
  if (Serial.available() == 0) return false;

  size_t len = Serial.readBytesUntil('\n', inputBuffer, BUF_SIZE - 1);
  inputBuffer[len] = '\0';

  if (len > 0 && inputBuffer[len - 1] == '\r') {
    inputBuffer[len - 1] = '\0';
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  Serial.println("BOOT");

  delay(100);
  motorDriver.Enable();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PEND_A, INPUT_PULLUP);
  pinMode(PEND_B, INPUT_PULLUP);

  uint8_t pendA = digitalRead(PEND_A);
  uint8_t pendB = digitalRead(PEND_B);
  prev_pendState = (pendA << 1) | pendB;

  attachInterrupt(digitalPinToInterrupt(PEND_A), pendulumEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PEND_B), pendulumEncoderISR, CHANGE);

  bool okA = setupExternalIrq(cartA_ctrl, cartA_cfg, CART_A_PIN, CART_A_IRQ, cartEncoderISR);
  bool okB = setupExternalIrq(cartB_ctrl, cartB_cfg, CART_B_PIN, CART_B_IRQ, cartEncoderISR);
  if (!okA) Serial.println("CART_IRQ_A_FAIL");
  if (!okB) Serial.println("CART_IRQ_B_FAIL");

  while (!okA || !okB) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // Initialize the cart's prev-state (pins are configured as inputs now).
  noInterrupts();
  enc_prevState = ((uint8_t)R_BSP_PinRead(CART_A_PIN) << 1) | (uint8_t)R_BSP_PinRead(CART_B_PIN);
  interrupts();

  bool ready = false;

  while (!ready) {
    Serial.println("REQ");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    
    if (readLineNonBlocking()) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
      // Expecting "RET,<value>"
      if (strncmp(inputBuffer, "RET", 3) == 0) {
        char* commaPos = strchr(inputBuffer, ',');
        if (commaPos != nullptr) {
          long saved = atol(commaPos + 1);
          noInterrupts();
          enc_count = saved;
          interrupts();
        }
        Serial.println("REC");
        ready = true;
      }
    }
  }

  pend_count = 0;
}

void loop() {
  // static uint32_t lastDump = 0;
  // if (millis() - lastDump >= 100) {
  //   lastDump = millis();
  //   uint8_t a = digitalRead(PEND_A);
  //   uint8_t b = digitalRead(PEND_B);
  //   Serial.print("PEND A="); Serial.print(a);
  //   Serial.print(" B="); Serial.print(b);
  //   Serial.print(" state="); Serial.print((a<<1)|b);
  //   Serial.print(" count="); Serial.println(pend_count);
  // }

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
      // E,pendulumCount,cartCount
      // (Python reads part[1] as the pendulum and part[2] as the cart)

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