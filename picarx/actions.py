from picarx_improved import Picarx
import atexit
import time

class Maneuvering(object):
    def __init__(self):
        self.px = Picarx()
        #self.px = px
        self.default_speed = 40
        self.default_steering = 20
        self.max_steering = 40
        self.pause = 1
        self.command_wait = 0.25
        atexit.register(self.cleanup)

    def calibrate_steering(self):
        self.px.forward(self.default_speed)
        time.sleep(self.pause)
        self.px.stop()


    def menu(self):
        while True:
            print("Welcome to the Picar menu!")
            print("0: Calibrate Steering")
            # print("1: Forward and Backward (with steering")
            # print("2: Parallel Parking")
            # print("3: K-turn")
            print("q: Quit")

            menu_option = input("Please select a maneuver or q to quit: ")
            if menu_option == "0":
                maneuvering.calibrate_steering()
            # elif menu_option == "1":
            #     maneuvering.forward_and_backward_with_steering()
            # elif menu_option == "2":
            #     maneuvering.parallel_parking()
            # elif menu_option == "3":
            #     maneuvering.k_turn()
            # elif menu_option == "q":
            #     return
            else:
                print("Invalid Selection")

    def cleanup(self):
        self.px.set_dir_servo_angle(0)
        self.px.stop

if __name__ == "__main__":
    maneuvering = Maneuvering()
    maneuvering.menu()