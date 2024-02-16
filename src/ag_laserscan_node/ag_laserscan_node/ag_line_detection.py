import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import math

class LineDetection(Node):
    def __init__(self):
        super().__init__('ag_line_detection')

        self.image_sub = self.create_subscription(
            Image,
            '/scan_to_image',
            self.lineseg_detect_callback,
            10
        )

        self.pub = self.create_publisher(Image, "/detected_lines", 10)
        
        self.bridge = CvBridge()

        self.min_dist = 120
        self.max_dist = 160

        self.row_one = 0
        self.row_two = 0

        self.lsd = cv2.createLineSegmentDetector(0)

    def distance_to_center(self, image_dim):
        h, w = image_dim

        rho = math.sqrt((h//2)**2 + (w//2)**2)

        return rho
    
    # Detects line using Line Segment Detector.
    def lineseg_detect_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        h, w, c = image.shape

        # image = image[h//44:3*h//4][w//44:3*w//4]
        # center_dist = math.sqrt((h//2)**2 + (w//2)**2)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.GaussianBlur(image, (5,5), 0)

        edges = cv2.Canny(gray, 25, 100)
        # gray = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
        
        lines = cv2.HoughLines(edges,1,np.pi/180, 38)
        
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                if(rho <= self.min_dist or rho>= self.max_dist):
                    continue
                
                theta_1 = 0
                theta_2 = 0
                theta = lines[i][0][1]
                if (rho > 200):
                    continue
                    self.row_two = rho
                    theta_2 = theta
                else:
                    self.row_one = rho
                    theta_1 = theta
                    
                # avg = (self.row_one+self.row_two)/2
                # avg_theta = (theta_1+theta_2)/2

                # print('Row 1: {}, Row 2: {}, avg: {}'.format(self.row_two, self.row_one, avg))

                a = math.cos(theta)
                print('Rho: {}, Theta: {}, Num: {}'.format(rho, theta, len(lines)))
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

                # avg_a = math.cos(avg_theta)
                # # print('Rho: {}, Theta: {}'.format(rho, theta))
                # avg_b = math.sin(avg_theta)
                # x0 = avg_a * avg
                # y0 = avg_b * avg
                # avg_pt1 = (int(x0 + 1000*(-avg_b)), int(y0 + 1000*(avg_a)))
                # avg_pt2 = (int(x0 - 1000*(-avg_b)), int(y0 - 1000*(avg_a)))

                cv2.line(image, pt1, pt2, (255,0,0), 1, cv2.LINE_AA)
                # cv2.line(image, avg_pt1, avg_pt2, (0,255,0), 1, cv2.LINE_AA)

        img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
		# Publish image
        self.pub.publish(img)
                
    # Detects line using hough transform.
    def line_segment_detect_callback_hough(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        h, w, c = image.shape

        # image = image[h//44:3*h//4][w//44:3*w//4]

        image = cv2.blur(image, (3,3))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, 50, 150)
        # gray = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
        
        lines = cv2.HoughLines(edges,1,np.pi/180, 15)
        
        if lines is not None:
            print('Length {}'.format(len(lines)))
            upper_arr_rho = []
            lower_arr_rho = []

            upper_arr_theta = []
            lower_arr_theta = []
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                if rho > self.distance_to_center((h,w)):
                    lower_arr_rho.append(rho)
                    lower_arr_theta.append(theta)
                else:
                    upper_arr_rho.append(rho)
                    upper_arr_theta.append(theta)
                # a = math.cos(theta)
                # print('Rho {}\nTheta{}'.format(rho, theta))
                # b = math.sin(theta)
                # x0 = a * rho
                # y0 = b * rho
                # pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                # pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                # cv2.line(image, pt1, pt2, (255,0,0), 1, cv2.LINE_AA)
            
            # if(upper_arr_rho):
            #     upper_avg_rho = sum(upper_arr_rho)/len(upper_arr_rho)
            #     upper_avg_theta = sum(upper_arr_theta)/len(upper_arr_theta)
            #     print('Upper Theta: {}'.format(upper_avg_theta))

            #     a = math.cos(upper_avg_theta)
            #     b = math.sin(upper_avg_theta)
            #     x0 = a * upper_avg_rho
            #     y0 = b * upper_avg_rho
            #     pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            #     pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

            #     cv2.line(image, pt1, pt2, (255,0,0), 1, cv2.LINE_AA)

            if(lower_arr_rho):
                lower_avg_rho = sum(lower_arr_rho)/len(lower_arr_rho)
                lower_avg_theta = sum(lower_arr_theta)/len(lower_arr_theta)
                print('Lower Theta: {}'.format(lower_avg_theta))

                a = math.cos(lower_avg_theta)
                b = math.sin(lower_avg_theta)
                x0 = a * lower_avg_rho
                y0 = b * lower_avg_rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

                cv2.line(image, pt1, pt2, (255,0,0), 1, cv2.LINE_AA)

        img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
		# Publish image
        self.pub.publish(img)

def main(args=None):
    rclpy.init(args=args)

    ag_line_detection = LineDetection()

    rclpy.spin(ag_line_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_line_detection.destroy_node()
    rclpy.shutdown()