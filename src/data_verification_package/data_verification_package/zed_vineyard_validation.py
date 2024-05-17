import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import cv2
import math
import pyzed.sl as sl

INVALID_VALUE = -88888888


class ZedVineyardValidation(Node):
    def __init__(self):
        super().__init__("zed_vineyard_validation")

        '''######## ZED CAMERA VARIABLES ########'''
        self.zed_camera = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER

        err = self.zed_camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(-1)

        self.image = sl.Mat(self.image_height, self.image_width, sl.MAT_TYPE.U8_C4)
        self.point_cloud = sl.Mat()
        self.image_ocv = None

        self.image_height = self.zed_camera.get_camera_information().camera_configuration.resolution.height
        self.image_width = self.zed_camera.get_camera_information().camera_configuration.resolution.width
        
        '''######## ROS INITIALIZATIONS ########'''
        self.true_xte_pub = self.create_publisher(Float64, "/ag/vineyard/true_xte", 10)
        self.true_heading_pub = self.create_publisher(Float64, "/ag/vineyard/true_heading", 10)
        self.masked_image_pub = self.create_publisher(Image, "video_frames", 10)

        timer_period = 0.1
        self.timer_pub = self.create_timer(timer_period, self.publish_xte_and_heading_callback)
        self.br = CvBridge()

        '''######## PROGRAM INITIALIZATIONS ########'''
        self.heading_avg = []

    def get_point_cloud_and_image(self) -> int:
        runtime_parameters = sl.RuntimeParameters()
        if self.zed_camera.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed_camera.retrieve_image(self.image, sl.VIEW.LEFT)
            self.image_ocv = self.image.get_data()
            self.zed_camera.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            return 1
        return INVALID_VALUE

    def get_process_image(self) -> np.matrix:
        image = self.image_ocv.copy()
        image = cv2.flip(image, 1)
        image = self.get_image_mask(image)
        return image

    def get_red_range(self):
        lower_red_ranges = [np.array([0, 70, 50]), np.array([170, 70, 50])]
        upper_red_ranges = [np.array([10, 255, 255]), np.array([180, 255, 255])]
        return (
            lower_red_ranges, 
            upper_red_ranges
            )

    # Retrieve the image mask for all red pixels in the image.
    def get_image_mask(self, image) -> np.matrix:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_range = self.get_red_range()
        lower_mask = cv2.inRange(image, color_range[0][0], color_range[1][0])
        upper_mask = cv2.inRange(image, color_range[0][1], color_range[1][1])
        return lower_mask + upper_mask

    # Optional: Applies the mask to the original image.
    def set_image_mask(self, image: np.matrix) -> np.matrix:
        mask = self.get_image_mask(image)
        return cv2.bitwise_and(image, image, mask=mask)

    def get_pixel_distance(self, x: int, y: int) -> tuple:
        if self.is_valid_value(x) and self.is_valid_value(y):
            point3D = self.point_cloud.get_value(x, y)
            px, py, pz = point3D[1][0], point3D[1][1], point3D[1][2]

            return (sl.ERROR_CODE.SUCCESS, math.sqrt(px ** 2 + py ** 2 + pz ** 2))
        
        return (sl.ERROR_CODE.FAILURE, INVALID_VALUE)

    def get_closest_pixel_to_value(self, array: list, value: int) -> int:
        pixel_locations = np.where(array == 255)[0]

        if len(pixel_locations) > 0:
            closest_index = np.argmin(np.absolute(pixel_locations - value))
            return int(pixel_locations[closest_index])
        return INVALID_VALUE

    def get_lateral_dist_from_center(self, d1: float, d2: float) -> float:
        return math.sqrt(max(d1, d2) ** 2 - min(d1, d2) ** 2)

    def moving_avg_filter(self, new_value, values, window_size=10) -> float:
        avg = 0
        if len(values) >= 10:
            values.pop(0)
            avg = sum(values) / window_size
        values.append(new_value)
        return avg

    def is_valid_value(self, value: float) -> bool:
        return (
            True
            if value != INVALID_VALUE
            and not (np.isnan(value))
            and not (math.isinf(value))
            else False
        )

    # Distance in meters.
    def get_crosstrack_error(self, center_dist: float=None) -> float:
        half_width = self.image_width // 2
        half_height = self.image_height // 2

        x = self.get_closest_pixel_to_value(self.image_ocv[half_height], half_width)
        strap_dist_code, strap_dist = self.get_pixel_distance(x, half_height)

        if not (center_dist):
            center_dist_code, center_dist = self.get_pixel_distance(half_width, half_height)

        if self.is_valid_value(center_dist) and self.is_valid_value(strap_dist):
            xte = self.get_lateral_dist_from_center(strap_dist, center_dist)
            if x < half_width:
                xte *= -1
            return xte    
        return INVALID_VALUE

    def get_line_slope(self) -> float:     
        py_1, py_2 = (self.image_height // 2) + 20, (self.image_height // 2)
        px_1, px_2 = self.get_closest_pixel_to_value(self.image_ocv[py_1], self.image_width // 2), self.get_closest_pixel_to_value(self.image_ocv[py_2], self.image_width // 2)

        try:
            if not (self.is_valid_value(px_1)) or not (self.is_valid_value(px_2)):
                raise Exception("Unable to compute slope. Line not detected.")
            print("Point 1: (%d, %d)\nPoint 2: (%d, %d)" % (px_1, py_1, px_2, py_2))
            slope = (float(px_2) - float(px_1)) / (float(py_2) - float(py_1))
        except ZeroDivisionError as divByZero:
            slope = 0
        except Exception as E:
            return INVALID_VALUE
        return slope

    def get_heading_from_line_slope(self) -> float:
        slope = self.get_line_slope()
        return -math.degrees(math.atan(slope)) if self.is_valid_value(slope) else slope

    def publish_xte_and_heading_callback(self) -> None:
        xte = Float64()
        heading = Float64()
        
        if self.get_point_cloud_and_image() == INVALID_VALUE:
            print("Unable to connect to zed camera.")
            return INVALID_VALUE

        self.image_ocv = self.get_process_image()

        xte.data = self.get_crosstrack_error()
        heading.data = self.get_heading_from_line_slope()

        if self.is_valid_value(xte.data) and self.is_valid_value(heading.data):
            heading_avg = self.moving_avg_filter(heading.data, self.heading_avg)
            print("Heading: %f\nXTE: %f" % (heading_avg, xte.data))
            self.true_xte_pub.publish(xte)
            self.true_heading_pub.publish(heading)

        self.masked_image_pub.publish(self.br.cv2_to_imgmsg(self.image_ocv))


def main(args=None):
    rclpy.init(args=args)

    zed_vineyard_validation = ZedVineyardValidation()

    rclpy.spin(zed_vineyard_validation)
    
    zed_vineyard_validation.destroy_node()
    rclpy.shutdown()
