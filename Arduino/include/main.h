#pragma once

#include <Arduino.h>

/*
  Sends pwm output with direction to motor driver

  value: 0-255
  direction: 1 = forward, 0 = reverse
*/
void drive_motor(int value, bool direction);