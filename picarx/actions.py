from picarx_improved import Picarx
import atexit
import time
import sys
class Maneuvering(object):
    def __init__(self):
        self.px = Picarx()
        #self.px = px
        self.speed = 80  # speed
        self.steering_angle = 30  #10 default angle
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
            print("Control Picar of death!!")
            print("1: Calibrate Steering")
            print("x: Quit")

            menu_option = input("select an action or quit: ")
            if menu_option == "1":
                maneuvering.calibrate_steering()
            elif menu_option == "x":
                raise SystemExit
            else:
                print("Invalid Selection")

    def cleanup(self):
        self.px.set_dir_servo_angle(0)
        self.px.stop

if __name__ == "__main__":
    maneuvering = Maneuvering()
    maneuvering.menu()