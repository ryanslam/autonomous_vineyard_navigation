import rclpy
from rclpy.node import Node
from ag_custom_message.msg import AgTruthData
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import cv2
import math
import pyzed.sl as sl

INVALID_VALUE = -88888888


class AgVineyardTruthDataVerification(Node):
    def __init__(self):
        super().__init__("ag_vineyard_verification")

        self.zed_camera = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER

        err = self.zed_camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(-1)

        self.image_height = (
            self.zed_camera.get_camera_information().camera_configuration.resolution.height
        )
        self.image_width = (
            self.zed_camera.get_camera_information().camera_configuration.resolution.width
        )

        self.truth_data_pub = self.create_publisher(
            AgTruthData, "/ag/vineyard/truth_data", 10
        )
        self.publisher_ = self.create_publisher(Image, "video_frames", 10)
        timer_period = 0.1
        self.timer_pub = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()

        self.image = sl.Mat(self.image_height, self.image_width, sl.MAT_TYPE.U8_C4)
        self.image_ocv = None
        self.depth_map = sl.Mat()

        self.cross_track_avg = []
        self.heading_avg = []

    def get_depth_map_and_image(self) -> int:
        runtime_parameters = sl.RuntimeParameters()
        if self.zed_camera.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed_camera.retrieve_image(self.image, sl.VIEW.LEFT)
            self.image_ocv = self.image.get_data()
            self.zed_camera.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
            return 1
        return INVALID_VALUE

    def get_pixel_distance(self, x, y) -> tuple:
        if self.is_valid_value(x) and self.is_valid_value(y):
            print("x: %f" % (self.depth_map.get_value(x, y)[1]))
            return self.depth_map.get_value(x, y)
        return (sl.ERROR_CODE.FAILURE, INVALID_VALUE)

    def get_lateral_dist_from_center(self, p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    # Distance in meters.
    def get_crosstrack_error(self, center_dist=None) -> float:
        half_width = self.image_width // 2
        half_height = self.image_height // 2
        if not (center_dist):
            center_dist_code, center_dist = self.get_pixel_distance(
                half_width, half_height
            )

        x = self.get_closest_pixel_to_value(self.image_ocv[half_height], half_width)

        strap_dist_code, strap_dist = self.get_pixel_distance(x, half_height)

        # print('Center Distance: (%s,%f)' % (center_dist_code, center_dist))
        # print('Strap Distance: (%s,%f)' % (strap_dist_code, strap_dist))
        if self.is_valid_value(center_dist) and self.is_valid_value(strap_dist):
            xte = self.get_lateral_dist_from_center(
                (x, strap_dist), (half_width, center_dist)
            )
            if x < half_width:
                xte *= -1
            return xte
        return INVALID_VALUE

    def moving_avg_filter(self, new_value, values, window_size=10):
        avg = 0
        if len(values) >= 10:
            values.pop(0)
            avg = sum(values) / window_size
        values.append(new_value)
        return avg

    def get_line_slope(self) -> float:
        py_1, py_2 = (self.image_height // 2) + 20, (self.image_height // 2)
        px_1, px_2 = self.get_closest_pixel_to_value(
            self.image_ocv[py_1], self.image_width // 2
        ), self.get_closest_pixel_to_value(self.image_ocv[py_2], self.image_width // 2)
        try:
            if not (self.is_valid_value(px_1)) or not (self.is_valid_value(px_2)):
                raise Exception("Unable to compute slope. Line not detected.")
            # if px_1 == px_2:
            #     raise Exception(ZeroDivisionError)
            print("Point 1: (%d, %d)\nPoint 2: (%d, %d)" % (px_1, py_1, px_2, py_2))
            slope = (float(px_2) - float(px_1)) / (float(py_2) - float(py_1))
        except ZeroDivisionError as divByZero:
            print(divByZero)
            slope = 0
        except Exception as E:
            print(E)
            slope = INVALID_VALUE
        return slope

    # Retrieve the image mask for all red pixels in the image.
    def get_image_mask(self, image) -> np.matrix:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red_ranges = [np.array([0, 70, 50]), np.array([170, 70, 50])]
        upper_red_ranges = [np.array([10, 255, 255]), np.array([180, 255, 255])]
        lower_mask = cv2.inRange(image, lower_red_ranges[0], upper_red_ranges[0])
        upper_mask = cv2.inRange(image, lower_red_ranges[1], upper_red_ranges[1])
        mask = lower_mask + upper_mask

        return mask

    # Optional: Applies the mask to the original image.
    def set_image_mask(self, image):
        mask = self.get_image_mask(image)
        return cv2.bitwise_and(image, image, mask=mask)

    def get_line_center(self, points):
        return sum(points) // 2

    def is_valid_value(self, value):
        return (
            True
            if value != INVALID_VALUE
            and not (np.isnan(value))
            and not (math.isinf(value))
            else False
        )

    def get_closest_pixel_to_value(self, array, value) -> int:
        pixel_locations = np.where(array == 255)[0]

        if len(pixel_locations) > 0:
            closest_index = np.argmin(np.absolute(pixel_locations - value))
            return int(pixel_locations[closest_index])

        return INVALID_VALUE

    def slope_to_heading(self, slope):
        return -math.degrees(math.atan(slope))

    def process_image(self) -> np.matrix:
        image = self.image_ocv.copy()
        image = cv2.flip(image, 1)
        image = self.get_image_mask(image)

        return image

    def timer_callback(self):

        if self.get_depth_map_and_image() == INVALID_VALUE:
            print("Unable to connect to zed camera.")
            return INVALID_VALUE

        processed_image = self.process_image()
        self.image_ocv = processed_image
        slope = self.get_line_slope()

        heading = self.slope_to_heading(slope)
        cross_track_err = self.get_crosstrack_error(0.71)

        if self.is_valid_value(slope) and self.is_valid_value(cross_track_err):
            heading_avg = self.moving_avg_filter(heading, self.heading_avg)
            cross_track_err = self.moving_avg_filter(
                cross_track_err, self.cross_track_avg
            )
            print("Heading: %f\nXTE: %f" % (heading_avg, cross_track_err))
        else:
            print(slope)

        # print("Heading: %f\nSlope: %f\nXTE: %f" % (heading_avg, slope, cross_track_err))

        self.publisher_.publish(self.br.cv2_to_imgmsg(processed_image))


def main(args=None):
    rclpy.init(args=args)

    ag_vineyard_verification = AgVineyardTruthDataVerification()

    rclpy.spin(ag_vineyard_verification)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_vineyard_verification.destroy_node()
    rclpy.shutdown()

    strap_dist_code, strap_dist = self.get_pixel_distance(x, half_height)

    # print('Center Distance: (%s,%f)' % (center_dist_code, center_dist))
    # print('Strap Distance: (%s,%f)'
