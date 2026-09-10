# cart_control.py
import os
import time

from config import COM_PORT
from simulation import constants
from simulation.lqg_controller import wrap_to_pi

class CartControl:
    def __init__(self, ser, enc_count_file):
      self.ser = ser
      self.enc_count_file = enc_count_file
      self.locked = True
      self.last_mag_val = None
      self.last_rotary_val = None
      self.last_speed_val = 0
      # read_encoders() below enforces its own hard wall-clock deadline via
      # in_waiting polling, so the port itself should never block waiting
      # for a newline. A near-zero timeout makes any stray blocking read()
      # calls (e.g. in handshake()) return immediately instead of stalling
      # for up to the old default of 1s.
      self.ser.timeout = 0
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
        """Non-blocking read: poll in_waiting and assemble the line
        ourselves so the 0.05s deadline below is an actual hard bound.
        (A plain self.ser.readline() would block up to the port's own
        timeout waiting for a newline, and the deadline check can't
        interrupt a call that's already blocking -- it only gets
        re-checked *between* calls.)
        """
        self.ser.reset_input_buffer()
        self.ser.write(b"R\n")

        deadline = time.time() + 0.05  # 50 ms hard timeout
        buf = bytearray()

        while time.time() < deadline:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
                while b'\n' in buf:
                    raw_line, _, buf = buf.partition(b'\n')
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    if line.startswith("E"):
                        part = line.split(',')
                        if len(part) == 3:
                            try:
                                mag_val = int(part[1])
                                rotary_val = int(part[2])
                                return mag_val, rotary_val
                            except ValueError:
                                pass
            else:
                time.sleep(0.001)  # don't busy-spin the CPU while waiting

        return None, None

    def read_state(self, mag_val=None, rotary_val=None):
        """Convert raw encoder counts into the physical units the simulation
        and LQG controller work in: (x, theta) in meters and radians.

        Pass in already-read (mag_val, rotary_val) to avoid a redundant
        serial round-trip when the caller already has a fresh reading
        (e.g. inside the real-time control loop); otherwise this reads the
        encoders itself.

        Returns (x, theta, mag_val, rotary_val), or (None, None, None, None)
        if the encoders couldn't be read.
        """
        if mag_val is None or rotary_val is None:
            mag_val, rotary_val = self.read_encoders()

        if mag_val is None or rotary_val is None:
            return None, None, None, None

        theta = wrap_to_pi((mag_val - constants.PENDULUM_ZERO_RAW) * constants.PENDULUM_RAW_TO_RADIANS)
        x = (rotary_val - constants.ROTARY_CENTER) * constants.METERS_PER_COUNT

        return x, theta, mag_val, rotary_val

    def enforce_limits(self, motor_val, rotary_val):
        """Cut motor command to zero if the soft-lock is engaged and the
        cart is being driven further past its end-of-travel encoder count."""
        if self.locked and ((motor_val > 0 and rotary_val < 3000) or (motor_val < 0 and rotary_val > 57000)):
            return 0
        return motor_val

    def force_to_motor(self, u):
        """Map a controller force command u (N, in [-u_max, u_max]) to the
        integer [-255, 255] motor command, compensating for the motor's
        deadband (no motion for |motor_val| < DEADBAND)."""
        if u == 0:
            return 0

        # Normalized magnitude in [0, 1]
        norm = min(abs(u) / constants.u_max, 1.0)

        # Rescale so any nonzero command lands outside the deadband,
        # spanning [DEADBAND, MOTOR_MAX] instead of [0, MOTOR_MAX]
        boosted = constants.DEADBAND + norm * (constants.MOTOR_MAX - constants.DEADBAND)

        sign = constants.MOTOR_SIGN * (1 if u > 0 else -1)
        motor_val = sign * boosted

        return int(max(min(motor_val, constants.MOTOR_MAX), -constants.MOTOR_MAX))

    def run_step(self, speed):
        mag_val, rotary_val = self.read_encoders()

        if mag_val is not None and rotary_val is not None:
            # if mag_val != self.last_mag_val or rotary_val != self.last_rotary_val:
            print("MAG: raw-", mag_val, " rad-", mag_val * constants.PENDULUM_RAW_TO_RADIANS,
                  " deg-", mag_val * constants.PENDULUM_RAW_TO_DEGREES, ", ROTARY: ", rotary_val)
                # self.last_mag_val = mag_val
                # self.last_rotary_val = rotary_val

            speed = self.enforce_limits(speed, rotary_val)

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