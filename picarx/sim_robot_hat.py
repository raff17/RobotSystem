import time
# !/usr/bin/env python3
import logging
import time

class Servo():
    MAX_PW = 2500
    MIN_PW = 500
    _freq = 50

    def __init__(self, pwm):
        pass

    # angle ranges -90 to 90 degrees
    def angle(self, angle):
        pass

    # pwm_value ranges MIN_PW 500 to MAX_PW 2500 degrees
    def set_pwm(self, pwm_value):
        pass


def test():
    pass


class fileDB(object):
    """A file based database.
    A file based database, read and write arguements in the specific file.
    """

    def __init__(self, db: str, mode: str = None, owner: str = None):
        '''Init the db_file is a file to save the datas.'''

        self.db = db
        # Check if db_file is existed, otherwise create one
        if self.db != None:
            self.file_check_create(db, mode, owner)
        else:
            raise ValueError('db: Missing file path parameter.')

    def file_check_create(self, file_path: str, mode: str = None, owner: str = None):
        pass

    def get(self, name, default_value=None):
        """Get value by data's name. Default value is for the arguemants do not exist"""
        return default_value

    def set(self, name, value):
        """Set value by data's name. Or create one if the arguement does not exist"""
        pass


class Pin():
    _dict = {
        "BOARD_TYPE": 12,
    }

    _dict_1 = {
        "D0": 17,
        "D1": 18,
        "D2": 27,
        "D3": 22,
        "D4": 23,
        "D5": 24,
        "D6": 25,
        "D7": 4,
        "D8": 5,
        "D9": 6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW": 19,
        "USER": 19,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 21,

    }

    _dict_2 = {
        "D0": 17,
        "D1": 4,  # Changed
        "D2": 27,
        "D3": 22,
        "D4": 23,
        "D5": 24,
        "D6": 25,  # Removed
        "D7": 4,  # Removed
        "D8": 5,  # Removed
        "D9": 6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW": 25,  # Changed
        "USER": 25,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 5,  # Changed
    }

    def __init__(self, *value):
        pass

    def check_board_type(self):
        pass

    def init(self, mode, pull=0):
        pass

    def dict(self, *_dict):
        pass

    def __call__(self, value):
        return 1

    def value(self, *value):
        return 1

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        pass

    def pull(self, *value):
        return self._pull

    def irq(self, handler=None, trigger=None, bouncetime=200):
        pass

    def name(self):
        return "Filler"

    def names(self):
        return ["Filler", "Filler"]

    class cpu(object):
        GPIO17 = 17
        GPIO18 = 18
        GPIO27 = 27
        GPIO22 = 22
        GPIO23 = 23
        GPIO24 = 24
        GPIO25 = 25
        GPIO26 = 26
        GPIO4 = 4
        GPIO5 = 5
        GPIO6 = 6
        GPIO12 = 12
        GPIO13 = 13
        GPIO19 = 19
        GPIO16 = 16
        GPIO26 = 26
        GPIO20 = 20
        GPIO21 = 21

        def __init__(self):
            pass


class PWM():
    REG_CHN = 0x20
    REG_FRE = 0x30
    REG_PSC = 0x40
    REG_ARR = 0x44

    ADDR = 0x14

    CLOCK = 72000000

    def __init__(self, channel, debug="critical"):
        pass

    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
        pass

    def prescaler(self, *prescaler):
        pass

    def period(self, *arr):
        pass

    def pulse_width(self, *pulse_width):
        pass

    def pulse_width_percent(self, *pulse_width_percent):
        pass


def test():
    pass


def test2():
    pass


class Ultrasonic():
    def __init__(self, trig, echo, timeout=0.02):
        self.trig = trig
        self.echo = echo
        self.timeout = timeout

    def _read(self):
        return 1

    def read(self, times=10):
        return 1


class DS18X20():
    def __init__(self, *args, **kargs):
        # self.pin = pin
        pass

    def scan(self):
        return []

    def convert_temp(self):
        pass

    def read_temp(self, rom):
        return 1

    def read(self, unit=1):
        # unit=0:  Fahrenheit
        # unit=1:  degree Celsius
        return [1]


class ADXL345():
    X = 0
    Y = 1
    Z = 2
    _REG_DATA_X = 0x32  # X-axis data 0 (6 bytes for X/Y/Z)
    _REG_DATA_Y = 0x34  # Y-axis data 0 (6 bytes for X/Y/Z)
    _REG_DATA_Z = 0x36  # Z-axis data 0 (6 bytes for X/Y/Z)
    _REG_POWER_CTL = 0x2D  # Power-saving features control
    _AXISES = [_REG_DATA_X, _REG_DATA_Y, _REG_DATA_Z]

    def __init__(self, address=0x53):
        pass

    def read(self, axis):
        return 1


class RGB_LED():
    def __init__(self, Rpin, Gpin, Bpin, common=1):
        pass

    def write(self, color):
        pass


class Buzzer():
    def __init__(self, pwm):
        self.pwm = pwm

    def on(self):
        pass

    def off(self):
        pass

    def freq(self, freq):
        pass

    def play(self, *args):
        return 10


class Sound():
    def __init__(self, pin):
        self.pin = pin

    def read_raw(self):
        return 1

    def read(self, times=50):
        return 1


class Joystick():
    import math
    THRESHOLD = 2047 / math.sqrt(2)

    def __init__(self, Xpin, Ypin, Btpin):
        pass

    @property
    def is_x_reversed(self):
        return 1

    @property
    def is_y_reversed(self):
        return 1

    @property
    def is_z_reversed(self):
        return 1

    @is_x_reversed.setter
    def is_x_reversed(self, value):
        pass

    @is_y_reversed.setter
    def is_y_reversed(self, value):
        pass

    @is_z_reversed.setter
    def is_z_reversed(self, value):
        pass

    def read(self, axis):
        return 1

    def read_status(self):
        return 'home'


class Grayscale_Module(object):
    def __init__(self, pin0, pin1, pin2, reference=1000):
        pass

    def get_line_status(self, fl_list):
        pass

    def get_grayscale_data(self):
        return [1, 1, 1]