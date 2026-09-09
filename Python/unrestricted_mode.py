import os
import sys
import time

import keyboard
import serial

from config import COM_PORT
from cart_control import CartControl

enc_count_file = os.path.join(os.path.dirname(__file__), "enc_counts.txt")

ser = serial.Serial(COM_PORT, 115200, timeout=1)
cart = CartControl(ser, enc_count_file)
cart.locked = False

enc_count = cart.handshake()
print("Loaded saved encoder count:", enc_count)

key_state = {}
def key_just_pressed(key):
    is_down = keyboard.is_pressed(key)
    was_down = key_state.get(key, False)
    key_state[key] = is_down
    return is_down and not was_down



while True:
    if keyboard.is_pressed('right'):
        cart.set_motor(-255)
    elif keyboard.is_pressed('left'):
        cart.set_motor(255)
    else:
        cart.set_motor(0)
    mag_val, rotary_val = cart.read_encoders()
    if mag_val is not None and rotary_val is not None:
        print("MAG: raw-", mag_val, ", ROTARY: ", rotary_val)
