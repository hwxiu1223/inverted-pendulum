import serial
import time

# Establish connection (Replace 'COM3' with your Arduino's port)
# Windows: 'COM3', 'COM4' etc. | Mac/Linux: '/dev/tty.usbmodem...' or '/dev/ttyUSB0'
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)

# Wait for connection to initialize
time.sleep(2)

def send_command(command):
    # Convert string to bytes and send to Arduino
    arduino.write(bytes(command, 'utf-8')) 
    time.sleep(0.1) # Give Arduino time to respond
    
    # Read the confirmation line back from Arduino
    if arduino.in_waiting:
        response = arduino.readline().decode('utf-8').strip()
        print(f"Arduino says: {response}")

try:
    while True:
        user_input = input("Enter 1 to turn LED ON, 0 to turn OFF (or 'exit'): ")
        if user_input.lower() == 'exit':
            break
        elif user_input in ['1', '0']:
            send_command(user_input)
        else:
            print("Invalid input.")
finally:
    arduino.close() # Always close the port when done
    print("Connection closed.")
