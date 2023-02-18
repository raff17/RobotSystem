import time
import statistics
from RosRos import Bus
from picarx_improved import Picarx

try:
    from robot_hat import ADC
except ImportError:
    print(
        "This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *


class Grayscale_Sensor:
    def __init__(self,
                 pin_1: str = "A0",
                 pin_2: str = "A1",
                 pin_3: str = "A2"):
        # grayscale sensors
        self.chn0 = ADC(pin_1)
        self.chn1 = ADC(pin_2)
        self.chn2 = ADC(pin_3)

        self.chn0_default = 0
        self.chn1_default = 0
        self.chn2_default = 0

    def read(self):
        cal = self.chn0.read()
        cal2 = self.chn1.read()
        cal3 = self.chn2.read()
        max = []
        max.append(cal)
        max.append(cal2)
        max.append(cal3)
        return max

    def calibrate_grayscale(self):
        self.chn0_default, self.chn1_default, self.chn2_default = self.read()

    def produce(self, sensor_bus: Bus, delay=0.5):
        while True:
            message = self.read()
            sensor_bus.set_message(message)
            time.sleep(delay)


class Grayscale_Interpreter(object):
    def __init__(self, sensitivity=0.5, polarity=True):
        if polarity:  # if black follow
            self.sensitivity = max(0, min(sensitivity, 1)) * 1
        else:  # if white follow
            self.sensitivity = max(0, min(sensitivity, 1)) * -1

        self.running = False

    def reading_direction(self, readings):
        noise_thresh = 8
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

    def produce_consume(self, sensor_bus: Bus, interpreter_bus: Bus, delay=0.05):
        self.running = True
        while self.running:
            message = sensor_bus.get_message()
            value = self.reading_direction(message)
            interpreter_bus.set_message(value)
            time.sleep(delay)


class Grayscale_Controller(object):
    def __init__(self, car: Picarx, scale=30, delay=0.05):
        self.scale = scale
        self.car = car
        self.running = False
        self.delay = delay

    def control(self, offset):
        steering_angle = offset * self.scale
        steer = self.car.set_dir_servo_angle(steering_angle)
        self.car.constant_move(20, steer)


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