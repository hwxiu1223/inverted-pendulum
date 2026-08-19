import serial
import time
import math
import keyboard
from config import COM_PORT

#### CONSTANTS ####
AS5600_RAW_TO_RADIANS = (math.pi * 2.0) / 4096
AS5600_RAW_TO_DEGREES = 360.0 / 4096

ser = serial.Serial(COM_PORT, 115200, timeout=1)
time.sleep(2) # Wait for Arduino to reset

saved_enc_count = 0
butt_pressed = False

locked = True;

# Track last printed encoder values so we only print on change
last_mag_val = None
last_rotary_val = None
last_speed_val = 0

def set_motor(speed):
    # Sends: M,150\n, between -255 and 255
    command = f"M,{speed}\n"
    ser.write(command.encode('utf-8'))

def read_encoders():
    # Request data
    ser.write(b"R\n")
    
    # Read the response: e.g., b'E,4095,1024\n'
    line = ser.readline().decode('utf-8').strip()
    
    if line.startswith("E"):
        parts = line.split(',')
        mag_val = int(parts[1])
        rotary_val = int(parts[2])
        return mag_val, rotary_val
    return None, None

def run_step(speed):
    global last_mag_val, last_rotary_val, last_speed_val

    mag_val, rotary_val = read_encoders()

    if mag_val is not None and rotary_val is not None:
        if mag_val != last_mag_val or rotary_val != last_rotary_val:
            print("MAG: raw-", mag_val, " rad-", mag_val * AS5600_RAW_TO_RADIANS, " deg-", mag_val * AS5600_RAW_TO_DEGREES, ", ROTARY: ", rotary_val)
            last_mag_val = mag_val
            last_rotary_val = rotary_val

    time.sleep(0.01)
    if locked and ((speed < 0 and rotary_val < 500) or (speed > 0 and rotary_val > 29500)):
        speed = 0

    set_motor(speed)

    if (speed != last_speed_val):
        print(f"Set motor: ", speed)
        
    last_speed_val = speed


def save_enc_val():
    global saved_enc_count
    mag_val, rot_val = read_encoders()
    count = rot_val

    if rot_val is not None:
        print("Saved enc count: ", count)
        return rot_val
    return None

while(1):
    if keyboard.is_pressed('right'):
        run_step(-255)
    elif keyboard.is_pressed('left'):
        run_step(255)
    else:
        run_step(0)

    if keyboard.is_pressed('u'):
        if locked:  
            locked = False
            print("!! Cart Unlocked !!")
        else:
            locked = True
            print("!! Cart Locked !!")
        time.sleep(0.10)
## saving the last encoder count
    if keyboard.is_pressed('s'):
        if not butt_pressed:
            saved_enc_count = save_enc_val()
            butt_pressed = True
    else:
        butt_pressed = False