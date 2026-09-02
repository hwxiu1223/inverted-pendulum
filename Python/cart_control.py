# cart_control.py
import time
import os
import constants
from config import COM_PORT

AS5600_RAW_TO_RADIANS, AS5600_RAW_TO_DEGREES = constants.AS5600_RAW_TO_RADIANS, constants.AS5600_RAW_TO_DEGREES

class CartControl:
    def __init__(self, ser, enc_count_file):
      self.ser = ser
      self.enc_count_file = enc_count_file
      self.locked = True
      self.last_mag_val = None
      self.last_rotary_val = None
      self.last_speed_val = 0
      if not os.path.exists(self.enc_count_file):
          with open(self.enc_count_file, "w") as file:
              file.write("enc_count: 0\n")
      self.saved_enc_count = self.load_enc_val()

    def load_enc_val(self):
        try:
            with open(self.enc_count_file, "r") as file:
                line = file.readline().strip()
            return int(line.split(":")[1].strip())
        except (FileNotFoundError, IndexError, ValueError):
            print("no prev value or error message")
            return 0

    def handshake(self):
        while True:
            response = self.ser.readline().decode('utf-8').strip()
            if response == 'REQ':
                print("Arduino request received")
                self.ser.write(f"RET,{self.saved_enc_count}\n".encode())
                print("Sending: ", f"RET, {self.saved_enc_count}")
                break

    def set_motor(self, speed):
        self.ser.write(f"M,{speed}\n".encode('utf-8'))

    def read_encoders(self):
        self.ser.write(b"R\n")
        line = self.ser.readline().decode('utf-8').strip()
        if line.startswith("E"):
            parts = line.split(',')
            return int(parts[1]), int(parts[2])
        return None, None

    def run_step(self, speed):
        mag_val, rotary_val = self.read_encoders()

        if mag_val is not None and rotary_val is not None:
            if mag_val != self.last_mag_val or rotary_val != self.last_rotary_val:
                print("MAG: raw-", mag_val, " rad-", mag_val * AS5600_RAW_TO_RADIANS,
                      " deg-", mag_val * AS5600_RAW_TO_DEGREES, ", ROTARY: ", rotary_val)
                self.last_mag_val = mag_val
                self.last_rotary_val = rotary_val

        time.sleep(0.01)
        if self.locked and ((speed > 0 and rotary_val < 1000) or (speed < 0 and rotary_val > 59000)):
            speed = 0

        self.set_motor(speed)
        if speed != self.last_speed_val:
            print("Set motor: ", speed)
        self.last_speed_val = speed

    def save_enc_val(self):
        _, rot_val = self.read_encoders()
        if rot_val is not None:
            print("Saved enc count: ", rot_val)
            with open(self.enc_count_file, "w") as file:
                file.write(f"enc_count: {rot_val}\n")
            self.saved_enc_count = rot_val
            return rot_val
        return None

    def center_cart(self):
        print("Centering...")
        while True:
            _, rot_val = self.read_encoders()
            if rot_val is None:
                self.set_motor(0)
                continue

            error = rot_val - 30000
            if abs(error) <= 100:
                self.set_motor(0)
                print("Cart is centered.")
                break

            self.set_motor(100 if error > 0 else -100)
            time.sleep(0.01)