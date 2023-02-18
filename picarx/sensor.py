from picarx_improved import Picarx
from readerwriterlock import rwlock
import time
import sys
from buss import Bus

sys.path.append("..")

try:
    from robot_hat import *
    from robot_hat import reset_mcu

    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print(
        "This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *


class Sensors(object):
    def __init__(self, pin0, pin1, pin2):
        self.chn0 = ADC(pin0)
        self.chn1 = ADC(pin1)
        self.chn2 = ADC(pin2)

        self.bus = Bus()
        self.delay = delay
        self.running = False

        self.chn0_default = 0
        self.chn1_default = 0
        self.chn2_default = 0

    def calibrate_grayscale(self):
        self.chn0_default, self.chn1_default, self.chn2_default = self.read()

    def read(self):
        adc_value_list = []
        adc_value_list.append(self.chn0.read())
        adc_value_list.append(self.chn1.read())
        adc_value_list.append(self.chn2.read())
        return adc_value_list

    def produce(self, bus, delay):
        self.running = True
        while self.running:
            self.bus.write(self.read())
            time.sleep(delay)


class Interpreter:
    def __init__(self, sensitivity=0.5, polarity=True):
        if polarity:  # if black follow
            self.sensitivity = max(0, min(sensitivity, 1)) * 1
        else:  # if white follow
            self.sensitivity = max(0, min(sensitivity, 1)) * -1
        self.bus = Bus()
        self.delay = delay
        self.running = False

    def reading_direction(self, readings):
        noise_thresh = 10
        left, middle, right = readings

        # Break early
        if abs((left - middle) - (right - middle)) < noise_thresh:
            return 0

        # Calculate the direction to turn
        if right - left > 0:
            direction = (middle - right) / (middle + right)
            direction *= -1
        else:
            direction = (middle - left) / (middle + left)

        return direction * self.sensitivity

    def produce_consume(self, sensor_bus, delay):
        self.running = True
        while self.running:
            self.bus.write(self.reading_direction(sensor_bus.read()))
            time.sleep(delay)


class Control:
    """Control interface used to drive the robot in a desired speed and direction."""

    def __init__(self, car: Picarx, scale):
        self.scale = scale
        self.car = car
        self.bus = Bus()
        self.delay = delay
        self.functioning = False

    def control(self, angle, speed=30):
        self.car.constant_move(speed, angle * self.scale)

    def consume(self, interpreter_bus, delay):
        self.functioning = True
        while self.functioning:
            self.control(interpreter_bus.read())
            time.sleep(delay)


if __name__ == "__main__":
    print()
#     car = Picarx()
#     sensor = Sensors("A0", "A1", "A2")
# print(sensor)
# Interpreter(sensor)
# d_or_w = input("dark or white target?: ")
# while True:
#     if d_or_w.lower() == "dark":
#         a = 1
#         # set the greater than or less than to flip
#     elif d_or_w.lower() == "white":
#         b = 1
#         # set greater than or less than to flip
#     else:
#         d_or_w = input("invalid target, Try again: ")
#
# print(sensor.read())
# print('sensor reading {}'.format(sensor.read()[0]))
# while True:
#     print(sensor.read())
#     if sensor.read()[0] < 150:
#         car.set_dir_servo_angle(-10)
#         car.forward(20)
#     if sensor.read()[2] < 150:
#         car.set_dir_servo_angle(10)
#         car.forward(20)
#     if sensor.read()[1] < 150:
#         car.set_dir_servo_angle(0)
#         car.forward(20)
#     else:
#         car.stop()
