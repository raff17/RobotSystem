from sensor import Sensors
from sensor import Interpreter
from sensor import Control
import time
from picarx_improved import Picarx


def follow_line(scale = 50):
    sensor = Sensors()
    input("Press enter to calibrate grayscale, make sure all sensors are on black")

    sensor.calibrate_grayscale()

    input("Press enter to calibrate grayscale, make sure all sensors are on black")
    interpreter = Interpreter()
    car = Picarx()
    controller = Control(car,scale)

    input("Press enter to start")

    # while(True):
    #     controller.control(interpreter.reading_direction(sensor.read()))
    #     time.sleep(0.1)

if __name__ == "__main__":
    follow_line()