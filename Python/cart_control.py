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

    def load_enc_val(self):
        try:
            with open(self.enc_count_file, "r") as file:
                line = file.readline().strip()
            return int(line.split(":")[1].strip())
        except (FileNotFoundError, IndexError, ValueError):
            print("no prev value or error message")
            return 0

    def handshake(self):
        sent_count = False
        print("Initiating Handshake")
        saved_enc_count = self.load_enc_val()
        while True:
            response = self.ser.readline().decode('utf-8').strip()
            if sent_count == False and response == 'REQ':
                sent_count = True 
                print("Arduino request received")
                self.ser.write(f"RET,{saved_enc_count}\n".encode())
                print("Sending: ", f"RET, {saved_enc_count}")
            elif response == 'REC':
                break
            time.sleep(0.1)
        return saved_enc_count

    def set_motor(self, speed):
        self.ser.write(f"M,{speed}\n".encode('utf-8'))

    def read_encoders(self):
        self.ser.write(b"R\n")
        # line = self.ser.readline().decode('utf-8').strip()
        # if line.startswith("E"):
        #     parts = line.split(',')
        #     return int(parts[1]), int(parts[2])
        # return None, None
        deadline = time.time() + 0.05 #0.5 second timeout
        while time.time() < deadline:
            line = self.ser.readline().decode('utf-8').strip()

            if not line:
                continue
            if line.startswith("E"):
                part = line.split(',')
                if len(part) == 3:
                    try:
                        mag_val = int(part[1])
                        rotary_val = int(part[2])
                        return mag_val, rotary_val
                    except ValueError:
                        pass
        return None, None
                

    def run_step(self, speed):
        mag_val, rotary_val = self.read_encoders()

        if mag_val is not None and rotary_val is not None:
            if mag_val != self.last_mag_val or rotary_val != self.last_rotary_val:
                print("MAG: raw-", mag_val, " rad-", mag_val * AS5600_RAW_TO_RADIANS,
                      " deg-", mag_val * AS5600_RAW_TO_DEGREES, ", ROTARY: ", rotary_val)
                self.last_mag_val = mag_val
                self.last_rotary_val = rotary_val

            if self.locked and ((speed > 0 and rotary_val < 3000) or (speed < 0 and rotary_val > 57000)):
                speed = 0
            
            self.set_motor(speed)
            if speed != self.last_speed_val:
                print("Set motor: ", speed)
            self.last_speed_val = speed

        else:
            print("!! Unable to get rotary encoder data !!")
            self.set_motor(0)

    def save_enc_val(self):
        _, rot_val = self.read_encoders()
        if rot_val is not None:
            print("Saved enc count: ", rot_val)
            with open(self.enc_count_file, "w") as file:
                file.write(f"enc_count: {rot_val}\n")
            return rot_val
        return None

    def reset_enc_val(self):
        print("Reset enc count.")
        with open(self.enc_count_file, "w") as file:
            file.write(f"enc_count: {0}\n")

        self.reset_arduino()
        time.sleep(0.5)
        self.handshake()
    
        return None

    def reset_arduino(self):
        self.ser.write(f"ST\n".encode('utf-8'))

    def center_cart(self):
        print("Centering...")

        while True:
            _, rot_val = self.read_encoders()
            if rot_val is None:
                print("Unable to read rotary encoder.")
                return None

            error = rot_val - 30000
            if abs(error) <= 10:
                break

            if abs(error) > 2000:
                speed = 255 if error > 0 else -255
            else:
                speed = (255 / 2000) * error
                if abs(speed) < 40:
                    speed = 40 if speed > 0 else -40

            self.set_motor(speed)
            time.sleep(0.01)

        self.set_motor(0)
        print("Cart is centered.")
        return None