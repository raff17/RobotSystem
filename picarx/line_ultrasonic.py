import os
import sys

sys.path.append("..")
from grayscale import Grayscale_Sensor, Grayscale_Controller, Grayscale_Interpreter
import ultrasonic
from picarx_improved import Picarx
from RosRos import (Bus, Consumer, ConsumerProducer, Producer, Timer, runConcurrently, )


def follow_line():
    car = Picarx()
    gs_sensor_bus = Bus()
    gs_control_bus = Bus()
    us_sensor_bus = Bus()
    us_control_bus = Bus()
    gs_sensor = Grayscale_Sensor()
    gs_interpreter = Grayscale_Interpreter()
    gs_controller = Grayscale_Controller(car)
    us_sensor = ultrasonic.Sensor(car=car)
    us_interpreter = ultrasonic.Interpreter(20)
    us_controller = ultrasonic.Control(car=car, speed=20)

    termination_bus = Bus()
    input("Press 'enter' to calibrate the sensor")
    gs_sensor.calibrate_grayscale()

    # Construct the sensor interfaces
    gs_prod = Producer(gs_sensor_bus, termination_buses=termination_bus, name="Greyscale_read",
                       delay=0.05,
                       )

    gs_prod_cons = ConsumerProducer(gs_interpreter.reading_direction, gs_sensor_bus, gs_control_bus,
                                    termination_buses=termination_bus,
                                    name="Greyscale Interpreter",
                                    delay=0.05,
                                    )

    gs_cons = Consumer(gs_controller.control, gs_control_bus,
                       termination_buses=termination_bus,
                       name="Greyscale Controller",
                       delay=0.05,
                       )

    us_prod = Producer(us_sensor.read, us_sensor_bus, termination_buses=termination_bus,
                       name="Ultrasonic Sensor",
                       delay=0.05,
                       )

    us_prod_cons = ConsumerProducer(us_interpreter.calculate_speed, us_sensor_bus, us_control_bus,
                                    termination_buses=termination_bus,
                                    name="Ultrasonic Interpreter",
                                    delay=0.05,
                                    )

    us_cons = Consumer(us_controller.control, us_control_bus, termination_buses=termination_bus,
                       name="Ultrasonic Controller",
                       delay=0.05,
                       )

    timer = Timer(termination_bus, termination_buses=termination_bus, delay=0.05, name="Timer",
                  duration=10,
                  )

    # sensor.calibrate()

    input("Press 'enter' to start line following")

    runConcurrently(
        [gs_prod, gs_prod_cons, gs_cons, us_prod, us_prod_cons, us_cons, timer]
    )


if __name__ == "__main__":
    follow_line()
