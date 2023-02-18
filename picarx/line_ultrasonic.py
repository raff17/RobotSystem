import os
import sys

sys.path.append("..")
from grayscale import Grayscale_Sensor, Grayscale_Controller, Grayscale_Interpreter
import ultrasonic
from picarx_improved import Picarx
from RosRos import (Bus, Consumer, ConsumerProducer, Producer, Timer, runConcurrently, )


def follow_line(config, user, scale: float = 50.0):
    gs_sensor_bus = Bus()
    gs_control_bus = Bus()
    us_sensor_bus = Bus()
    us_control_bus = Bus()

    termination_bus = Bus()

    car = Picarx(config, user)

    # Construct the sensor interfaces
    sensor = photosensor.Sensor()
    gs_interpreter = photosensor.Interpreter(polarity=True)
    gs_controller = photosensor.Control(car, scale)
    us_sensor = ultrasonic.Sensor(car)
    us_interpreter = ultrasonic.Interpreter(30)
    us_controller = ultrasonic.Control(car, 50)

    gs_prod = Producer(
        sensor.read,
        gs_sensor_bus,
        termination_buses=termination_bus,
        name="Greyscale Sensor",
        delay=0.05,
    )

    gs_prod_cons = ConsumerProducer(
        gs_interpreter.detect_direction,
        gs_sensor_bus,
        gs_control_bus,
        termination_buses=termination_bus,
        name="Greyscale Interpreter",
        delay=0.05,
    )

    gs_cons = Consumer(
        gs_controller.control,
        gs_control_bus,
        termination_buses=termination_bus,
        name="Greyscale Controller",
        delay=0.05,
    )

    us_prod = Producer(
        us_sensor.read,
        us_sensor_bus,
        termination_buses=termination_bus,
        name="Ultrasonic Sensor",
        delay=0.05,
    )

    us_prod_cons = ConsumerProducer(
        us_interpreter.calculate_speed,
        us_sensor_bus,
        us_control_bus,
        termination_buses=termination_bus,
        name="Ultrasonic Interpreter",
        delay=0.05,
    )

    us_cons = Consumer(
        us_controller.control,
        us_control_bus,
        termination_buses=termination_bus,
        name="Ultrasonic Controller",
        delay=0.05,
    )

    timer = Timer(
        termination_bus,
        termination_buses=termination_bus,
        delay=0.05,
        name="Timer",
        duration=10,
    )

    input("Press 'enter' to calibrate the sensor")

    sensor.calibrate()

    input("Press 'enter' to start line following")

    runConcurrently(
        [gs_prod, gs_prod_cons, gs_cons, us_prod, us_prod_cons, us_cons, timer]
    )


if __name__ == "__main__":
    # Disable security checks - this was written by the SunFounder folks
    user = os.popen("echo ${SUDO_USER:-$LOGNAME}").readline().strip()  # nosec
    home = os.popen(f"getent passwd {user} | cut -d: -f 6").readline().strip()  # nosec
    config = f"{home}/.config/picar-x/picar-x.conf"

    follow_line(config, user)
