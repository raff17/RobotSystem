import math
import os
import sys
from typing import Any

import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

sys.path.append("..")

from picarx_improved import Picarx  # noqa


class LaneDetector:

    def __init__(self) -> None:
        """Create a new lane detection interface."""
        ...


    def detect_edges(self, frame: cv2.Mat) -> Any:

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        edges = cv2.Canny(mask, 200, 400)

        return edges

    def detect_line_segments(self, cropped_edges: cv2.Mat) -> Any:

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

    def average_slope_intercept(
        self, frame: cv2.Mat, line_segments: list[cv2.Mat] | None
    ) -> list[list[list[int]]]:

        lane_lines: list[list[list[int]]] = []

        if line_segments is None:
            return lane_lines

        _, width, _ = frame.shape
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

    def make_points(self, frame: cv2.Mat, line: np.ndarray) -> list[list[int]]:

        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height
        y2 = int(y1 * 1 / 2)

        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

        return [[x1, y1, x2, y2]]

    def region_of_interest(self, canny: cv2.Mat) -> cv2.Mat:

        height, width = canny.shape
        mask = np.zeros_like(canny)

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
        masked_image = cv2.bitwise_and(canny, mask)

        return masked_image

    def compute_steering_angle(
        self, frame: cv2.Mat, lane_lines: list[list[list[int]]]
    ) -> float:

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

    def stabilize_steering_angle(
        self,
        curr_steering_angle: float,
        new_steering_angle: int,
        num_of_lane_lines: int,
        max_angle_deviation_two_lines: int = 5,
        max_angle_deviation_one_lane: int = 1,
    ) -> float:

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


def lane_following(
    config: str,
    user: str,
    resolution: tuple[int, int] = (640, 480),
    framerate: int = 24,
) -> None:

    detector = LaneDetector()
    car = Picarx(config, user)

    camera = PiCamera()

    camera.resolution = resolution
    camera.framerate = framerate

    raw = PiRGBArray(camera, size=resolution)

    for frame in camera.capture_continuous(raw, format="bgr", use_video_port=True):
        frame = frame.array

        edges = detector.detect_edges(frame)
        roi = detector.region_of_interest(edges)
        segments = detector.detect_line_segments(roi)
        lane_lines = detector.average_slope_intercept(frame, segments)

        angle = detector.compute_steering_angle(frame, lane_lines)

        car.drive(0.3, angle - 90)

        # Exit if the `esc` key is pressed
        key = cv2.waitKey(1) & 0xFF

        if key == 28:
            cv2.destroyAllWindows()
            camera.close()
            car.stop()

            break


if __name__ == "__main__":
    user = os.popen("echo ${SUDO_USER:-$LOGNAME}").readline().strip()  # nosec
    home = os.popen(f"getent passwd {user} | cut -d: -f 6").readline().strip()  # nosec
    config = f"{home}/.config/picar-x/picar-x.conf"

    lane_following(config, user)