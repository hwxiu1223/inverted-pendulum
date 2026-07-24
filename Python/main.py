import serial
import time
import math

#### CONSTANTS ####
AS5600_RAW_TO_RADIANS = (math.pi * 2.0) / 4096
AS5600_RAW_TO_DEGREES = 360.0 / 4096

# Connect to Uno R4 (Check your specific COM port / /dev/tty...)
ser = serial.Serial('COM3', 115200, timeout=1)
time.sleep(2) # Wait for Arduino to reset

def set_motor(speed):
    # Sends: M,150\n
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


while(1):
    mag_val, rotary_val = read_encoders()

    print("MAG: raw-", mag_val, " rad-", mag_val * AS5600_RAW_TO_RADIANS, " deg-", mag_val * AS5600_RAW_TO_DEGREES, ", ROTARY: ", rotary_val)
    time.sleep(1)