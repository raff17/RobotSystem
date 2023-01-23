from picarx_improved import Picarx
import atexit
import time

class Maneuvering(object):
    def __init__(self):
        self.px = Picarx()
        #self.px = px
        self.speed = 25  # speed
        self.steering_angle = 5  # default angle
        self.max_angle = 40  # max angle
        self.pause = .5
        self.command_wait = 0.25
        atexit.register(self.cleanup)

    def calibrate_steering(self):
        self.px.forward(self.speed)
        time.sleep(self.pause)
        self.px.stop()

    def forward_and_back_with_angles(self):
        forward_angle = input("insert a forward steering angle between [0-40]: ")
        while True:
            if forward_angle.isdigit():
                break
            else:
                forward_angle = input("invalid angle, Try again: ")

        # forward
        self.px.set_dir_servo_angle(forward_angle)
        time.sleep(self.command_wait)
        self.px.forward(self.speed)
        time.sleep(self.pause)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)


    def menu(self):
        while True:
            print("Control Picar of death!!")
            print("1: Calibrate Steering")
            print("2: forward_and_back_with_angles")
            print("x: Quit")

            menu_option = input("select an action or quit: ")
            if menu_option == "1":
                maneuvering.calibrate_steering()
            elif menu_option == "2":
                maneuvering.forward_and_back_with_angles()
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