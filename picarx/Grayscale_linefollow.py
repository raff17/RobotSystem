from sensor import Sensors
from sensor import Interpreter
from sensor import Control
import time
from picarx_improved import Picarx


def follow_line(scale=50):
    sensor = Sensors("A0", "A1", "A2")
    input("Press enter to calibrate grayscale, make sure all sensors are on black")

    sensor.calibrate_grayscale()

    follow = input("Following black? enter [true] to follow white enter [false]")
    while True:
        if follow.lower() == "true":
            break
        elif follow.lower() == "false":
            break
        else:
            follow = input("invalid color, Try again: ")

    interpreter = Interpreter(polarity=follow)
    car = Picarx()
    controller = Control(car, scale)

    input("Press enter to start")

    while (True):
        controller.control(interpreter.reading_direction(interpreter, Sensors.read()))
        time.sleep(0.1)


if __name__ == "__main__":
    follow_line()
