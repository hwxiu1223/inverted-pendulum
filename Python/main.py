    import os
    import time
    import keyboard
    import serial
    import sys
    from config import COM_PORT
    from cart_control import CartControl

    enc_count_file = os.path.join(os.path.dirname(__file__), "enc_counts.txt")

    ser = serial.Serial(COM_PORT, 115200, timeout=1)
    cart = CartControl(ser, enc_count_file)

    enc_count = cart.handshake()
    print("Loaded saved encoder count:", enc_count)

    key_state = {}
    control_active = False

    def key_just_pressed(key):
        is_down = keyboard.is_pressed(key)
        was_down = key_state.get(key, False)
        key_state[key] = is_down
        return is_down and not was_down

    while True:
        if key_just_pressed('p'):
        control_active = not control_active
        if control_active:
            print("starting control")
        else:
            cart.set_motor(0)
            print("stopping control")

        if control_active:
            # TODO: replace with actual balancing/control logic
            pass

        if key_just_pressed('s'):
                cart.save_enc_val()
        
        if key_just_pressed('m'):
            cart.center_cart()
        
        if key_just_pressed('r'):
            cart.set_motor(0)
            cart.reset_enc_val()
        
        if key_just_pressed('space'):
            cart.set_motor(0)
            print("Cart stopped, saving encoder count...")
            time.sleep(1)
            saved = cart.save_enc_val()
            print("Stopped and saved encoder count:", saved)

            cart.reset_arduino()
            time.sleep(1)
        
            sys.exit()
            break