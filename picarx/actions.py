from picarx_improved import Picarx
import atexit
import time
import string

class Maneuvering(object):
    def __init__(self):
        self.px = Picarx()
        #self.px = px
        self.speed = 40  # speed
        self.steering_angle = 5  # default angle
        self.max_angle = 35  # max angle
        self.pause = .5
        self.command_wait = 0.25
        atexit.register(self.cleanup)

    def calibrate_steering(self):
        self.px.forward(self.speed)
        time.sleep(self.pause)
        self.px.stop()

    def forward_and_back_with_angles(self):
        forward_angle = input("insert a forward steering angle between [0-35]: ")
        direction = input("Left or Right: ")
        while True:
            if forward_angle.isdigit():  # checks to see if int is int, if True breaks
                forward_angle = int(forward_angle)
                break
            else:
                forward_angle = input("invalid angle, Try again: ")

        # forward
        while True:
            if direction.lower() == "left" or direction.lower() == "right":  # checks correct direction
                break
            else:
                direction = input("Incorrect direction{left or right}: ")

        if direction.lower() == "right":
            self.px.set_dir_servo_angle(forward_angle)
            time.sleep(self.command_wait)
            self.px.forward(self.speed)
            time.sleep(self.pause)
            self.px.stop()
            time.sleep(self.command_wait)
            self.px.set_dir_servo_angle(0)
            time.sleep(self.command_wait)
        elif direction.lower() == "left":
            self.px.set_dir_servo_angle(-forward_angle)
            time.sleep(self.command_wait)
            self.px.forward(self.speed)
            time.sleep(self.pause)
            self.px.stop()
            time.sleep(self.command_wait)
            self.px.set_dir_servo_angle(0)
            time.sleep(self.command_wait)



    def parallel_park(self):

        direction = input("Left or Right: ")
        while True:
            if direction.lower() == "right":
                break
            elif direction.lower() == "left":
                break
            else:
                direction = input("invalid direction, Try again: ")

        # starts moving forward
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)
        self.px.forward(self.speed)
        time.sleep(1)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)

        # turns left or right
        if direction.lower() == "right":
            self.px.set_dir_servo_angle(35)
        else:
            self.px.set_dir_servo_angle(-35)
        # move back while turn
        time.sleep(self.command_wait)
        self.px.backward(self.speed)
        time.sleep(self.pause)
        self.px.stop()
        time.sleep(self.command_wait)
        self.px.set_dir_servo_angle(0)
        time.sleep(self.command_wait)

        # fix turn and move back





    def menu(self):
        while True:
            print("Control Picar of death!!")
            print("1: Calibrate Steering")
            print("2: forward_and_back_with_angles")
            print("3: parallel_park")
            print("x: Quit")

            menu_option = input("select an action or quit: ")
            if menu_option == "1":
                maneuvering.calibrate_steering()
            elif menu_option == "2":
                maneuvering.forward_and_back_with_angles()
            elif menu_option == "3":
                maneuvering.parallel_park()
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