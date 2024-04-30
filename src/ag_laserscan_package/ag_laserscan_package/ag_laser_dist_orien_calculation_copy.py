import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import math

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
        
        self.pub = self.create_publisher(Pose2D, "/ag/percept/dist_and_angle", 10)
        self.cross_track_err_pub = self.create_publisher(Float32, "/ag/percept/cross_track_error", 10)

        self.offset = 45

        self.angle_arr_left = []
        self.angle_arr_right = []
        
    def moving_avg_filter(self, new_value, values, window_size=10):
        avg = 0
        if(len(values) >= 10):
            values.pop(0)
            avg = sum(values)/window_size
        values.append(new_value)
        return avg

    # Retrieve ratios of the rover based on minimum value scan.
    def retrieve_dist_and_angle_callback(self, scan):
        msg = Pose2D()
        cross_track_err_msg = Float32()
        
        laser_ranges = scan.ranges
        print("Max Range: %f", (laser_ranges(179)))

        min_angle_diff_l = 90-(np.argmin(np.array(laser_ranges[30:150]))+30)
        min_angle_l_avg = self.moving_avg_filter(min_angle_diff_l, self.angle_arr_left)
        print("unfiltered angle left: ", min_angle_diff_l)
        print("filtered angle left: ", min_angle_l_avg)

        min_angle_diff_r = 270-(np.argmin(np.array(laser_ranges[210:330]))+210)
        min_angle_r_avg = self.moving_avg_filter(min_angle_diff_r, self.angle_arr_right)
        print("unfiltered angle right: ", min_angle_diff_r)
        print("filtered angle right: ", min_angle_r_avg)
        
        left = min(laser_ranges[45:135])
        right = min(laser_ranges[225:315])

        cross_track_error = left-right
        cross_track_err_msg.data = cross_track_error

        self.cross_track_err_pub.publish(cross_track_err_msg)

        print("l: ", min(laser_ranges[45:135]))
        print("r: ", min(laser_ranges[225:315]))
 
        # print(dist_and_angle)
        # json_string = json.dumps(dist_and_angle)
        # msg.data = json_string
        if math.isinf(min(laser_ranges[45:135])):
            msg.x = 0.0
        else:
            msg.x = min(laser_ranges[45:135])
            
        if math.isinf(min(laser_ranges[225:315])):
            msg.y = 0.0
        else:
            msg.y = min(laser_ranges[225:315])

        msg.theta = (min_angle_l_avg + min_angle_r_avg)/2
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