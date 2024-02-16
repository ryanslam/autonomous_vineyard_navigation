import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
import math
import json

class AlterLaserScan(Node):

    def __init__(self):
        super().__init__('altered_laser_scan')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription_2 = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.alter_scan_callback,
            qos_profile=qos_profile,)
        
        self.pub_scan = self.create_publisher(LaserScan, "ag/perception/altered_scan", 10)

    def alter_scan_callback(self, msg):
        scan = msg
        ranges = scan.ranges

        i = 0
        while(i<len(ranges)):
            ranges[i] = ranges[i+1] = min(ranges[i], ranges[i+1])
        i+=2

        self.pub_scan.publish(scan)
        

def main(args=None):
    rclpy.init(args=args)

    altered_laser_scan = AlterLaserScan()

    rclpy.spin(altered_laser_scan)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    altered_laser_scan.destroy_node()
    rclpy.shutdown()