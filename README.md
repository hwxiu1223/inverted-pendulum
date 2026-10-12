# Inverted Pendulum Project

A cart-and-rail system that balances a pendulum upright.

By Justin Chung, Henry Xiu, Evan Yeung, and Micah Hauser

![3D Model of full cart and rail system](/Website/Images/cart-assembly.jpg)

## Hardware
- Arduino UNO R4
- BTS7960 motor driver
- 2 rotary encoders: one for cart position, one for pendulum angle
- 1 DC 12v 1000 RPM Gear Motor

## How It Works
- **Setup:** The Arduino is an interface for the motor and rotary encoders, while Python runs the control loop.
- **Protocol:** We built a simple call-and-response protocol over serial. Python sends `R` and the Arduino replies `E,<pendulum>,<cart>`. Python sends `M,<speed>` to drive the motor.
- **Cart position:** Rotary encoders only track relative movement, so we save the count when we stop and reload it at startup. We picked a vendor encoder over a homemade axle because its machined tolerances are more reliable than what we could make.
- **Interrupts:** The pendulum encoder uses `attachInterrupt()`. The UNO R4 officially exposes only 2 interrupt pins, so the cart encoder talks to the RA4M1's external interrupt controller directly.
- **Control:** We tuned the control system in MATLAB first, then ported it to Python so it runs in real time.
- **Soft walls:** The cart count can drift, so the code eases the cart to a stop before it reaches the end of the track.

## Testing
- To check the Arduino interface, we wrote a manual mode (`manual.py`) that drives the cart with the keyboard and prints encoder values.
- We also wrote scripts to measure the motor deadband and the pendulum's true decay.

## Lessons Learned
1. **AS5600 magnetic encoder:** It worked fine at first, but once we read it alongside the cart encoder, the system lagged and hung for up to 10 s. We suspect I2C bus interference or flooding, but never confirmed it. We switched to a second rotary encoder. *Lesson: research how a peripheral works before committing to it.*
2. **Interrupt pins:** The Arduino library only natively supported enough interrupt request pins for 1 rotary encoder. To integrate the second, we accessed the ICU registers directly. The rotary encoder that used our custom ICU code would drift, so the pendulum keeps the native interrupts and the cart uses the ICU. Small drift matters less for position, which is why we added soft limits. *Lesson: abstraction layers can hide what the hardware can actually do.*
3. **Unresolved issues:** We still got occasional bad data. Twisting each encoder phase wire with a ground wire helped a bit. Swing-up would also sometimes work for a while, then stop and just swing the pendulum back and forth. We suspect breadboard wiring/EMI for the first problem and an overheating processor for the second, but haven't confirmed either.

## Build/Run
1. Flash `main.cpp` to the Arduino (libraries: BTS7960; UNO R4 core with `IRQManager`).
2. Set `COM_PORT` in `config.py`.
3. Install dependencies: `pip install pyserial keyboard numpy scipy`
4. Run `python main.py` for the controller or `python manual.py` for manual mode.
5. Keys: `p` start/stop, `m` center cart, `s` save encoder count, `r` reset, `space` quit.
