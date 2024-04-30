import rclpy
from rclpy.node import Node
from ag_custom_message.msg import AgTruthData

import numpy as np
import cv2
import math

class AgVineyardTruthDataVerification:

    def __init__(self):
        super().__init__('ag_vineyard_verification')

        self.camera = cv2.VideoCapture(0)

        self.truth_data_pub = self.create_publisher(AgTruthData, '/ag/vineyard/truth_data', 10)
        self.timer_period = self.create_timer(timer_period, self.timer_callback)

        timer_period = 0.1
        
    # Retrieve the image mask for all red pixels in the image.
    def get_image_mask(self, image) -> np.matrix:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red_ranges = [np.array([0, 100, 20]), np.array([160, 100, 20])]
        upper_red_ranges = [np.array([10, 25, 255]), np.array([179, 255, 255])]
        
        lower_mask = cv2.inRange(image, lower_red_ranges[0], upper_red_ranges[0])

        upper_mask = cv2.inRange(image, lower_red_ranges[1], upper_red_ranges[1])

        mask = lower_mask + upper_mask

        return mask
    
    # Optional: Applies the mask to the original image.
    def set_image_mask(self, image) -> np.matrix:
        mask = self.get_image_mask(image)
        return cv2.bitwise_and(image, image, mask=mask)
    
    def get_pixel_location(self, array):
        pixel_locations = np.where(array==255)
        first_occurance = pixel_locations[0]
        last_occurance = pixel_locations[-1]

        return [first_occurance, last_occurance]
    
    def get_line_center(self, points):
        return sum(points)//2

    def get_crosstrack_error(self, image):
        height, width = image.shape[:2]
        center_points = self.get_pixel_location(image[height//2])
        actual_line_center = self.get_line_center(center_points)
        return actual_line_center - width//2

    def get_line_slope(self, image):
        height, _ = image.shape[:2]

        y_one = height//2
        y_two = (height//2) - 5

        x_one, _ = self.get_pixel_location(image[y_one])
        x_two, _ = self.get_pixel_location(image[y_two])

        return (float(y_two)-float(y_one))/(float(x_two)-float(x_one))
    
    def slope_to_heading(self, slope):
        return -math.degrees(math.atan(slope))

    def timer_callback(self):
        msg = AgTruthData()

        ret, frame = self.camera.read()

        image = frame.copy()
        image_mask = self.get_image_mask(image)
        slope = self.get_line_slope(image_mask)

        msg.true_crosstrack_error = self.get_crosstrack_error(image_mask)
        msg.true_heading = self.slope_to_heading(slope)

        self.truth_data_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    ag_vineyard_verification = AgVineyardTruthDataVerification()

    rclpy.spin(ag_vineyard_verification)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_vineyard_verification.destroy_node()
    rclpy.shutdown()