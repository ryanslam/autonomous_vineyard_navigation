# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import LaserScan, Image
# from cv_bridge import CvBridge, CvBridgeError

# import cv2
# import numpy as np
# import math

# class WaypointCalculation(Node):

#     def __init__(self):
#         super().__init__('ag_waypoint_calculation')

#         self.image_sub = self.create_subscription(
#             Image,
#             '/scan_to_image',
#             self.retrieve_dist_and_angle_callback,
#             10
#         )

#         def retrieve_dist_and_angle_callback(self,):
