from picarx_improved import Picarx
import time
import sys

sys.path.append("..")

try:
    from robot_hat import *
    from robot_hat import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot_hat is not present). Shadowing hardware calls with substitute functions ")
    from sim_robot_hat import *


class Sensors(object):
    def __init__(self,pin0,pin1,pin2):
        self.chn0 = ADC(pin0)
        self.chn1 = ADC(pin1)
        self.chn2 = ADC(pin2)

    def read(self):
        adc_value_list = []
        adc_value_list.append(self.chn0.read())
        adc_value_list.append(self.chn1.read())
        adc_value_list.append(self.chn2.read())
        return adc_value_list

if __name__ == "__main__":
    car = Picarx()
    sensor = Sensors("A0","A1","A2")
    print(sensor.read())
    print('sensor reading {}'.format(sensor.read()[0]))
    while True:
        print(sensor.read())
        if sensor.read()[0] < 150:
            car.set_dir_servo_angle(-10)
            car.forward(20)
        if sensor.read()[2] < 150:
            car.set_dir_servo_angle(10)
            car.forward(20)
        if sensor.read()[1] < 150:
            car.set_dir_servo_angle(0)
            car.forward(20)
        else:
            car.stop()






