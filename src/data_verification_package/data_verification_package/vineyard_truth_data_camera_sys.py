import rclpy
from rclpy.node import Node
from ag_custom_message.msg import AgTruthData
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import cv2
import math

class AgVineyardTruthDataVerification(Node):

    def __init__(self):
        super().__init__('ag_vineyard_verification')

        self.camera = cv2.VideoCapture(0)

        self.truth_data_pub = self.create_publisher(AgTruthData, '/ag/vineyard/truth_data', 10)
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        timer_period = 0.1
        self.timer_pub = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()

        self.ret = None
        self.frame = None
        
    # Retrieve the image mask for all red pixels in the image.
    def get_image_mask(self, image) -> np.matrix:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red_ranges = [np.array([0, 100, 20]), np.array([173, 100, 100])]
        upper_red_ranges = [np.array([10, 25, 255]), np.array([179, 255, 255])]
        
        lower_mask = cv2.inRange(image, lower_red_ranges[0], upper_red_ranges[0])

        upper_mask = cv2.inRange(image, lower_red_ranges[1], upper_red_ranges[1])

        mask = lower_mask + upper_mask

        return mask
    
    # Optional: Applies the mask to the original image.
    def set_image_mask(self, image):
        mask = self.get_image_mask(image)
        return cv2.bitwise_and(image, image, mask=mask)
    
    def get_pixel_location(self, array):
        pixel_locations = np.where(array==255)
        try:
            first_occurance = pixel_locations[0][0]
            last_occurance = pixel_locations[0][-1]
        except:
            print("Insufficient red lines to reliably determine information.")
            return [None, None]

        return [first_occurance, last_occurance]
    
    def get_line_center(self, points):
        return sum(points)//2

    def get_crosstrack_error(self, image):
        height, width = image.shape[:2]
        center_points = self.get_pixel_location(image[height//2])

        if(self.is_valid_value(center_points[0]) and self.is_valid_value(center_points[1])):
            actual_line_center = self.get_line_center(center_points)
            return float(actual_line_center - width//2)
        else:
            print("Unable to reliably calculate crosstrack error.")
            return

    def is_valid_value(self, value):
        return value != None

    def get_line_slope(self, image) -> float:
        height, _, = image.shape[:2]

        y_one, y_two = height//2 + 3, (height//2) - 3

        x_one, _ = self.get_pixel_location(image[y_one])
        x_two, _ = self.get_pixel_location(image[y_two])

        slope = None

        if((self.is_valid_value(x_one) and self.is_valid_value(x_two)) and (x_one != x_two)):
            slope = (float(y_two)-float(y_one))/(float(x_two)-float(x_one))
        
        return slope
    
    def slope_to_heading(self, slope):
        return -math.degrees(math.atan(slope))
    
    def process_image(self) -> np.matrix:
        image = self.frame.copy()
        image = cv2.flip(image, 1)
        image = self.get_image_mask(image)
        # image = cv2.GaussianBlur(image, (5, 5), sigmaX=0)

        return image

    def timer_callback(self):
        msg = AgTruthData()
        msg.header = "Vision Verification"
        msg.name_id = 1

        self.ret, self.frame = self.camera.read()
        
        processed_image = self.process_image()
        slope = self.get_line_slope(processed_image)
        cross_track_err = self.get_crosstrack_error(processed_image)
        
        if(self.is_valid_value(slope) and self.is_valid_value(cross_track_err)):
            print('Slope: %f\nCrosstrack Err: %f' % (slope, cross_track_err))
            msg.true_crosstrack_error = cross_track_err
            msg.true_heading = self.slope_to_heading(slope)
            self.truth_data_pub.publish(msg)

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