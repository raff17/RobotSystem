from picarx_improved import Picarx
import atexit
import time

class Maneuvering(object):
    def __init__(self):
        self.px = Picarx()
        #self.px = px
        self.speed = 20  # speed
        self.steering_angle = 10  # default angle
        self.max_angle = 40  # max angle
        self.pause = 1
        self.command_wait = 0.25
        atexit.register(self.cleanup)

    def calibrate_steering(self):
        self.px.forward(self.speed)
        time.sleep(self.pause)
        self.px.stop()


    def menu(self):
        while True:
            print("Welcome to the Picar menu!")
            print("1: Calibrate Steering")
            print("q: Quit")

            menu_option = input("Please select a maneuver or q to quit: ")
            if menu_option == "0":
                maneuvering.calibrate_steering()
            else:
                print("Invalid Selection")

    def cleanup(self):
        self.px.set_dir_servo_angle(0)
        self.px.stop

if __name__ == "__main__":
    maneuvering = Maneuvering()
    maneuvering.menu()