import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
import math
import json

class AgDistOrienCalc(Node):

    def __init__(self):
        super().__init__('ag_laser_dist_orien_calculation')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.retrieve_dist_and_angle_callback,
            qos_profile=qos_profile,)
        
        self.subscription_2 = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.alter_scan_callback,
            qos_profile=qos_profile,)
        
        self.pub = self.create_publisher(String, "/ag/percept/dist_and_angle", 10)

        self.pub_scan = self.create_publisher(LaserScan, "ag/perception/altered_scan", 10)

        self.offset = 45

    def alter_scan_callback(self, msg):
        scan1 = LaserScan
        scan2 = msg

        #for i in scan2.ranges:
        scan2.ranges = [5.1]*360

        # scan1 = msg

        # for i in range(36):
        #     try:
        #         min_val = min(msg.ranges[i, i+10])
        #         for j in range(10):
        #             scan2.ranges[i+j] = min_val
        #         print('test')
        #     except:
        #         continue
        #     i*=10

        self.pub_scan.publish(scan2)
    
    def calculate_slope(self, point_angles, ranges, side, dist):
        window_size = 5
        if(side == 'l'):
            point_one_index = 90 + point_angles
            point_two_index = 90 - point_angles
        elif(side == 'r'):
            point_one_index = 270 + point_angles
            point_two_index = 270 - point_angles
        
        point_one = ranges[point_one_index-window_size:point_one_index+window_size]
        point_two = ranges[point_two_index-window_size:point_two_index+window_size]

        point_one = [val for val in point_one if not(math.isnan(val)) and val <= dist]
        point_two = [val for val in point_two if not(math.isnan(val)) and val <= dist]
        try:
            if(len(point_one) > 0):
                point_one_avg = sum(point_one)/len(point_one)

            if (len(point_two) > 0):
                point_two_avg = sum(point_two)/len(point_two)

        
            return math.atan(point_one_avg/point_two_avg)
        except:
            return None

    # Retrieve ratios of the rover based on minimum value scan.
    def retrieve_dist_and_angle_callback(self, scan):
        msg = String()
        window_size = 5
        laser_ranges = scan.ranges

        # Retrieve areas of interest dist.
        l_ranges_front = laser_ranges[135-window_size:135+window_size]
        l_ranges_back = laser_ranges[45-window_size:45+window_size]
        r_ranges_front = laser_ranges[225-window_size:225+window_size]
        r_ranges_back = laser_ranges[315-window_size:315+window_size]

        # Filter the ranges.
        l_ranges_front = [val for val in l_ranges_front if not(math.isnan(val)) and val <= 2.5]
        l_ranges_back = [val for val in l_ranges_back if not(math.isnan(val)) and val <= 2.5]
        r_ranges_front = [val for val in r_ranges_front if not(math.isnan(val)) and val <= 2.5]
        r_ranges_back = [val for val in r_ranges_back if not(math.isnan(val)) and val <= 2.5]

        l_ranges_front = None
        l_ranges_back = None
        r_ranges_front = None
        r_ranges_back = None

        if(len(l_ranges_front) > 0):
            # print(l_ranges)
            l_front_min = min(l_ranges_front)

        if(len(l_ranges_back) > 0):
            # print(l_ranges)
            l_back_min = min(l_ranges_back)

        if(len(r_ranges_front) > 0):
            # print(r_ranges)
            r_front_min = min(r_ranges_front)

        if(len(r_ranges_front) > 0):
            # print(r_ranges)
            r_back_min = min(r_ranges_back)       

        if(l_front_min and r_back_min):
            ratio_cross_1 = l_front_min/r_back_min
            
        if(r_front_min and l_back_min):
            ratio_cross_2 = r_front_min/l_back_min

        if(ratio_cross_1 and ratio_cross_2):
            ratio_r = r_ranges_front/r_ranges_back
            ratio_l = l_ranges_front/l_ranges_back
 
        dist_and_angle = {
            'cross_1': ratio_cross_1,
            'cross_2': ratio_cross_2,
            'ratio_right': ratio_r,
            'ratio_left': ratio_l
        }

        print(" ")
        json_string = json.dumps(dist_and_angle)
        msg.data = json_string

        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    ag_dist_orien = AgDistOrienCalc()

    rclpy.spin(ag_dist_orien)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_dist_orien.destroy_node()
    rclpy.shutdown()