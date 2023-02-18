from picarx_improved import Picarx


class Sensor:
    """Sensor class used to get ultrasonic sensor readings."""

    def __init__(self, car: Picarx):
        self.car = car

    def read(self):
        return max(self.car.get_distance(), 0)


class Interpreter:
    """Interprets sensor readings obtained from the ultrasonic sensor for control."""

    def __init__(self, min_distance: float):
        self.min_distance = min_distance

    def calculate_speed(self, distance: float):
        return 0 if distance < self.min_distance else 1


class Control:
    """Controls the speed of the robot."""

    def __init__(self, car: Picarx, speed: float):
        self.speed = speed
        self.car = car

    def control(self, speed_scalar: float):
        self.car.constant_move(self.speed * speed_scalar)