# cart_control.py
import time
import os
import constants
from config import COM_PORT

AS5600_RAW_TO_RADIANS, AS5600_RAW_TO_DEGREES = constants.AS5600_RAW_TO_RADIANS, constants.AS5600_RAW_TO_DEGREES

# ---------------------------------------------------------------------------
# Calibration constants for the LQG bridge. MEASURE THESE ON YOUR RIG -- the
# values below are placeholders and the loop will misbehave until you set
# them for real.
# ---------------------------------------------------------------------------
ROTARY_CENTER_COUNT = 30000        # rotary count at true track center (matches center_cart())
ROTARY_COUNTS_PER_METER = 59000/0.99 # TODO: measure -- rotary counts per 1 m of cart travel
ROTARY_SIGN = 1                    # flip to -1 if +x should be the other physical direction

THETA_OFFSET_RAD = 0.0             # TODO: measure -- raw*AS5600_RAW_TO_RADIANS with pendulum hanging straight down
THETA_SIGN = 1                     # flip to -1 if the encoder counts the wrong rotational direction

MOTOR_DEADBAND = 40                # same deadband compensation already used in center_cart()


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
                sent_count == True
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
        self.ser.reset_input_buffer()
        self.ser.write(b"R\n")
        line = self.ser.readline().decode('utf-8').strip()
        if line.startswith("E"):
            parts = line.split(',')
            return int(parts[1]), int(parts[2])
        return None, None

    def read_state(self):
        """
        Read the raw encoders and convert to the (x, theta) units MATLAB
        expects: x in meters (0 = track center), theta in radians
        (0 = pendulum hanging straight down, matching cartpend.m/pendsys.m).

        Returns (None, None) if a sensor read failed.
        """
        mag_val, rotary_val = self.read_encoders()
        if mag_val is None or rotary_val is None:
            return None, None

        x = ROTARY_SIGN * (rotary_val - ROTARY_CENTER_COUNT) / ROTARY_COUNTS_PER_METER
        theta = THETA_SIGN * (mag_val * AS5600_RAW_TO_RADIANS) - THETA_OFFSET_RAD
        return x, theta

    def send_force(self, u, u_max):
        """
        Convert a controller force command (N, from hybrid_controller_lqg.m)
        into a motor speed command (-255..255) and send it.

        This is a linear approximation (speed ~ u / u_max * 255) with the
        same static-friction deadband already used in center_cart(). It has
        NOT been validated against real motor dynamics -- expect to tune it.
        """
        speed = 255.0 * u / u_max
        if 0 < abs(speed) < MOTOR_DEADBAND:
            speed = MOTOR_DEADBAND if speed > 0 else -MOTOR_DEADBAND
        speed = max(min(speed, 255), -255)
        self.set_motor(speed)

    def run_step(self, speed, poll_encoders_every=5):
        # Use the last known rotary reading for the lock check so a motor
        # command never has to wait on a fresh (and possibly slow/blocked)
        # encoder round trip. `rotary_val is not None` also prevents a
        # TypeError on the very first call, before any reading exists.
        rotary_val = self.last_rotary_val
        if self.locked and rotary_val is not None and (
            (speed > 0 and rotary_val < 3000) or (speed < 0 and rotary_val > 57000)
        ):
            speed = 0

        else:
            print("!! Unable to get rotary encoder data !!")
            self.set_motor(0)

        # Poll the encoders far less often than we send motor commands, so
        # keypress responsiveness is never gated behind a serial round trip.
        self._poll_counter = getattr(self, '_poll_counter', 0) + 1
        if self._poll_counter % poll_encoders_every == 0:
            mag_val, rotary_val = self.read_encoders()

            if mag_val is not None and rotary_val is not None:
                if mag_val != self.last_mag_val or rotary_val != self.last_rotary_val:
                    print("MAG: raw-", mag_val, " rad-", mag_val * AS5600_RAW_TO_RADIANS,
                          " deg-", mag_val * AS5600_RAW_TO_DEGREES, ", ROTARY: ", rotary_val)
                    self.last_mag_val = mag_val
                    self.last_rotary_val = rotary_val

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