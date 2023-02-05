import math
import os
import sys
from typing import Any
import time
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

sys.path.append("..")

from picarx_improved import Picarx  # noqa


class LaneDetector:
    """Interface used to detect lanes on a Picarx."""

    def __init__(self):
        """Create a new lane detection interface."""
        ...

    def detect_edges(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        edges = cv2.Canny(mask, 200, 400)

        return edges

    def detect_line_segments(self, cropped_edges):
        """
        Detect the line segments from an image filtered to display only the edges.
        :param cropped_edges: image processed by edge detection algorithm
        :type cropped_edges: cv2.Mat
        :return: line segments from the image
        :rtype: Any
        """
        rho = 1
        angle = np.pi / 180
        min_threshold = 10

        line_segments = cv2.HoughLinesP(
            cropped_edges,
            rho,
            angle,
            min_threshold,
            np.array([]),
            minLineLength=8,
            maxLineGap=4,
        )

        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        """
        Calculate the intercept between line segments.
        :param frame: camera frame
        :type frame: cv2.Mat
        :param line_segments: list of line segments
        :type line_segments: list[cv2.Mat] | None
        :return: identified lane lines
        :rtype: list[list[list[int]]]
        """
        lane_lines = []

        if line_segments is None:
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1 / 3

        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue

                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]

                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)

        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)

        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        return lane_lines

    def make_points(self, frame, line):
        """
        Get a list of points representing the current line segment.
        This will be the start and end points of a line segment.
        :param frame: camera frame; used to determine the position of the line in the
            frame space.
        :type frame: cv2.Mat
        :param line: line whose points should be obtained
        :type line: np.ndarray
        :return: start and end points for the line using the camera frame dimensions
        :rtype: list[list[int]]
        """
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height
        y2 = int(y1 * 1 / 2)

        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

        return [[x1, y1, x2, y2]]

    def detect_lane(self, frame):

        edges = self.detect_edges(frame)
        cropped_edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.average_slope_intercept(frame, line_segments)

        return lane_lines

    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=2):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    def region_of_interest(self, edge):

        height, width = edge.shape
        mask = np.zeros_like(edge)

        # only focus bottom half of the screen
        polygon = np.array(
            [
                [
                    (0, height * 1 / 2),
                    (width, height * 1 / 2),
                    (width, height),
                    (0, height),
                ]
            ],
            np.int32,
        )

        cv2.fillPoly(mask, polygon, 255)
        masked_image = cv2.bitwise_and(edge, mask)

        return masked_image

    def display_heading_line(self,frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        # Note: the steering angle of:
        # 0-89 degree: turn left
        # 90 degree: going straight
        # 91-180 degree: turn right
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image

    def compute_steering_angle(self, frame, lane_lines):
        """
        Calculate the steering angle (degrees) from the frame and lane lines.
        :param frame: current camera frame
        :type frame: cv2.Mat
        :param lane_lines: detected lane lines
        :type lane_lines: list[list[list[int]]]
        :return: calculated steering angle
        :rtype: float
        """
        if len(lane_lines) == 0:
            return -90

        height, width, _ = frame.shape
        if len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset: float = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]

            camera_mid_offset_percent = 0.02
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid

        y_offset = int(height / 2)

        # angle (in radian) to center vertical line
        angle_to_mid_radian = math.atan(x_offset / y_offset)

        # Convert to degrees
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        steering_angle = angle_to_mid_deg + 90

        return steering_angle

    def stabilize_steering_angle(self,
                                 curr_steering_angle: float,
                                 new_steering_angle: int,
                                 num_of_lane_lines: int,
                                 max_angle_deviation_two_lines: int = 5,
                                 max_angle_deviation_one_lane: int = 1):
        """
        Stabilize the steering angle.
        This essentially clamps the steering angle to prevent sharp turns.
        :param curr_steering_angle: current steering angle
        :type curr_steering_angle: float
        :param new_steering_angle: new steering angle to drive at
        :type new_steering_angle: int
        :param num_of_lane_lines: number of line lanes
        :type num_of_lane_lines: int
        :param max_angle_deviation_two_lines: maximum deviation between the two angles,
            defaults to 5
        :type max_angle_deviation_two_lines: int, optional
        :param max_angle_deviation_one_lane: maximum angle deviation in a lane,
            defaults to 1
        :type max_angle_deviation_one_lane: int, optional
        :return: stabilized steering angle
        :rtype: float
        """
        if num_of_lane_lines == 2:
            # if both lane lines detected, then we can deviate more
            max_angle_deviation = max_angle_deviation_two_lines
        else:
            max_angle_deviation = max_angle_deviation_one_lane

        angle_deviation = new_steering_angle - curr_steering_angle

        if abs(angle_deviation) > max_angle_deviation:
            stabilized_steering_angle = int(
                curr_steering_angle
                + max_angle_deviation * angle_deviation / abs(angle_deviation)
            )
        else:
            stabilized_steering_angle = new_steering_angle

        return stabilized_steering_angle


def lane_following(resolution=(640,480), framerate=24):
    print("start color detect")
    detector = LaneDetector()
    car = Picarx()

    camera = PiCamera()
    camera.resolution = resolution
    camera.framerate = framerate
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    time.sleep(2)
    # Continuously capture camera frames from the feed and process them for lane
    # following
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame.array

        edges = detector.detect_edges(frame)
        roi = detector.region_of_interest(edges)
        segments = detector.detect_line_segments(roi)
        lane_lines = detector.average_slope_intercept(frame, segments)
        angle = detector.compute_steering_angle(frame, lane_lines)
        # car.drive(0.3, angle - 90)
        # car.forward(3)
        cv2.imshow("video", edges)
        cv2.imshow("mask", roi)
        car.constant_move(10, angle/5)
        # Exit if the `esc` key is pressed
        rawCapture.truncate(0)  # Release cache
        k = cv2.waitKey(1) & 0xFF
        # 27 is the ESC key, which means that if you press the ESC key to exit
        if k == 27:
            break

    print('quit ...')
    cv2.destroyAllWindows()
    camera.close()


if __name__ == "__main__":
    lane_following()
