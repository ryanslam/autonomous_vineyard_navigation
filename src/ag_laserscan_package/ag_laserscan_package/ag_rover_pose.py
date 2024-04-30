import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
import math
import json

class AgPose2D(Node):

    def __init__(self):
        super().__init__('ag_pose_2d')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.x_component_sub = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.get_components,
            qos_profile=qos_profile,)
        
        self.rover_pose_publisher = self.create_publisher(Pose2D, 'ag/percept/pose', 10)

        self.x_component = 0
        self.y_component = 0

    def is_valid_point(self, distance):
        if(not(np.isinf(distance))):
            return True
        
    def get_x_component(self, scan):
        window_size = 5
        x_comp = 0.0

        x1 = scan[354:359]
        x2 = scan[0:5]
        x_comp_window = x1 + x2
        x_comp_window = [distance for distance in x_comp_window if self.is_valid_point(distance)]

        if(x_comp_window):
            x_comp = sum(x_comp_window)/len(x_comp_window)
        return x_comp
    
    def get_y_component(self, scan):
        y_component = 0.0
        if not(math.isinf(min(scan[225:315]))):
            y_component = min(scan[225:315])
        return y_component
        
    def get_components(self, msg):
        rover_pose = Pose2D()

        scan = msg.ranges
        rover_pose.x = self.get_x_component(scan)
        rover_pose.y = self.get_y_component(scan)
        
        self.rover_pose_publisher.publish(rover_pose)

def main(args=None):
    rclpy.init(args=args)

    ag_rover_pose = AgPose2D()

    rclpy.spin(ag_rover_pose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_rover_pose.destroy_node()
    rclpy.shutdown()