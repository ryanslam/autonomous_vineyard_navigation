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


class AgDistOrienCalc(Node):

    def __init__(self):
        super().__init__("ag_laser_dist_orien_calculation")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.retrieve_dist_and_angle_callback,
            qos_profile=qos_profile,
        )

        self.pub = self.create_publisher(Pose2D, "/ag/percept/dist_and_angle", 10)
        self.float_pub = self.create_publisher(
            Float32, "/ag/percept/lidar_confidence", 10
        )
        self.cross_track_err_pub = self.create_publisher(
            Float32, "/ag/percept/cross_track_error", 10
        )

        self.offset = 45

        self.actual_dist = [0.8, 1, 1.2, 1.4, 1.6, 1.8]
        self.measured_ratio = [0.7, 0.9, 1.1, 1.3, 1.5, 1.9]

        self.confidence_matrix = np.zeros((2, 8))

        self.angle_arr_left = []
        self.angle_arr_right = []

        # self.actual_dist =      [0.7, 0.9, 1.1, 1.35, 1.5, 1.7, 1.9, 2.1]
        # self.measured_ratio =   [0.27, 0.5, 0.69, 1.1, 1.3, 1.5, 2.8, 2.9]

    def calculate_slope(self, point_angles, ranges, side, dist):
        window_size = 5
        if side == "l":
            point_one_index = 90 + point_angles
            point_two_index = 90 - point_angles
        elif side == "r":
            point_one_index = 270 + point_angles
            point_two_index = 270 - point_angles

        point_one = ranges[
            point_one_index - window_size : point_one_index + window_size
        ]
        point_two = ranges[
            point_two_index - window_size : point_two_index + window_size
        ]

        point_one = [val for val in point_one if not (math.isnan(val)) and val <= dist]
        point_two = [val for val in point_two if not (math.isnan(val)) and val <= dist]
        try:
            if len(point_one) > 0:
                point_one_avg = sum(point_one) / len(point_one)

            if len(point_two) > 0:
                point_two_avg = sum(point_two) / len(point_two)

            return math.atan(point_one_avg / point_two_avg)
        except:
            return None

    def moving_avg_filter(self, new_value, values, window_size=10):
        avg = 0
        if len(values) >= 10:
            values.pop(0)
            avg = sum(values) / window_size
        values.append(new_value)
        return avg

    # Retrieve ratios of the rover based on minimum value scan.
    def retrieve_dist_and_angle_callback(self, scan):
        msg = Pose2D()
        float_msg = Float32()
        cross_track_err_msg = Float32()

        window_size = 10
        laser_ranges = scan.ranges

        # Retrieve areas of interest dist.
        l_ranges_front = laser_ranges[135 - window_size : 135 + window_size]
        l_ranges_back = laser_ranges[45 - window_size : 45 + window_size]
        r_ranges_front = laser_ranges[225 - window_size : 225 + window_size]
        r_ranges_back = laser_ranges[315 - window_size : 315 + window_size]

        # Filter the ranges.
        l_ranges_front = [
            val for val in l_ranges_front if not (math.isnan(val)) and val <= 2.5
        ]
        l_ranges_back = [
            val for val in l_ranges_back if not (math.isnan(val)) and val <= 2.5
        ]
        r_ranges_front = [
            val for val in r_ranges_front if not (math.isnan(val)) and val <= 2.5
        ]
        r_ranges_back = [
            val for val in r_ranges_back if not (math.isnan(val)) and val <= 2.5
        ]

        # l_ranges_front = None
        # l_ranges_back = None
        # r_ranges_front = None
        # r_ranges_back = None
        l_front_min = None
        l_back_min = None
        r_front_min = None
        r_back_min = None

        d1_l1 = None
        d1_l3 = None
        d2_l2 = None
        d2_l4 = None

        print("")
        if len(l_ranges_front) > 0:
            l_front_min = min(l_ranges_front)
            l_front_argmin = math.radians(np.argmin(np.array(l_ranges_front)) + 1)
            d1_l3 = l_front_min * math.cos(l_front_argmin)

        if len(l_ranges_back) > 0:
            l_back_min = min(l_ranges_back)
            l_back_argmin = math.radians(np.argmin(np.array(l_ranges_back)) + 1)
            d1_l1 = l_back_min * math.cos(l_back_argmin)

        if len(r_ranges_front) > 0:
            r_front_min = min(r_ranges_front)
            r_front_argmin = math.radians(np.argmin(np.array(r_ranges_front)) + 1)
            d2_l4 = r_front_min * math.cos(r_front_argmin)

        if len(r_ranges_back) > 0:
            r_back_min = min(r_ranges_back)
            r_back_argmin = math.radians(np.argmin(np.array(r_ranges_back)) + 1)
            d2_l2 = r_back_min * math.cos(r_back_argmin)

        l1l2 = None
        l3l4 = None

        if l_ranges_back and r_ranges_back:
            l1l2 = d1_l1 / d2_l2

        if l_ranges_front and r_ranges_front:
            l3l4 = d1_l3 / d2_l4

        min_angle_diff_l = 90 - (np.argmin(np.array(laser_ranges[30:150])) + 30)
        min_angle_l_avg = self.moving_avg_filter(min_angle_diff_l, self.angle_arr_left)
        print("unfiltered angle left: ", min_angle_diff_l)
        print("filtered angle left: ", min_angle_l_avg)

        min_angle_diff_r = 270 - (np.argmin(np.array(laser_ranges[210:330])) + 210)
        min_angle_r_avg = self.moving_avg_filter(min_angle_diff_r, self.angle_arr_right)
        print("unfiltered angle right: ", min_angle_diff_r)
        print("filtered angle right: ", min_angle_r_avg)

        # test_val =  2.875
        k_arr = self.obtain_gain()
        est_k_front = self.scale_gains(k_arr, l1l2)
        est_k_back = self.scale_gains(k_arr, l3l4)

        angle_l, min_l = self.find_angle(laser_ranges, 0, 180, 30)
        angle_r, min_r = self.find_angle(laser_ranges, 180, 360, 30)
        # print(angle_l, angle_r)
        # print("Min_l: ", min_l)
        # print("Min_r: ", min_r)

        self.confidence_matrix[0, 0] = min_l
        self.confidence_matrix[0, 1] = min_r
        # print(self.confidence_matrix)
        if min_r + min_l / 2 >= 1 and min_r + min_l / 2 <= 2:
            float_msg.data = 1.0
        else:
            float_msg.data = 0.0

        self.float_pub.publish(float_msg)

        # print(np.argmin(np.array(laser_ranges[88:93])))
        # print('0-180:' + f'{sum(laser_ranges[88:93])/len(laser_ranges[88:93]):.2f}')
        # print('180-360:' + f'{sum(laser_ranges[268:273])/len(laser_ranges[268:273]):.2f}')

        # left = sum(laser_ranges[88:93])/len(laser_ranges[88:93])
        # right = sum(laser_ranges[268:273])/len(laser_ranges[268:273])
        left = min(laser_ranges[45:135])
        right = min(laser_ranges[225:315])
        cross_track_error = left - right
        cross_track_err_msg.data = cross_track_error
        self.cross_track_err_pub.publish(cross_track_err_msg)

        print("l: ", min(laser_ranges[45:135]))
        print("r: ", min(laser_ranges[225:315]))

        dist_and_angle = {
            "l1l2": l1l2,
            "l3l4": l3l4,
            "d1_l1": d1_l1,
            "d1_l3": d1_l3,
            "d2_l2": d2_l2,
            "d2_l4": d2_l4,
        }

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

        msg.theta = (min_angle_l_avg + min_angle_r_avg) / 2
        self.pub.publish(msg)

    def find_angle_confidence(self, prev_angle_arr, angle_arr, max_angle_dif):
        left_angle_diff = angle_arr[0] - prev_angle_arr[0]
        right_angle_diff = angle_arr[1] - prev_angle_arr[1]

        left_angle_confidence = left_angle_diff / max_angle_dif
        right_angle_confidence = right_angle_diff / max_angle_dif

    def find_angle(self, laser_scan, min_angle_range, max_angle_range, angle_offset):
        angle = 0
        ratio_arr = []
        for i in range(min_angle_range, max_angle_range - angle_offset):
            ratio = laser_scan[i] / laser_scan[i + angle_offset]
            ratio_arr.append(ratio)

            # if(prev_ratio > min(abs(ratio-1), abs(prev_ratio-1))):
            #     angle = i
            #     prev_ratio = ratio
        ratio_arr = np.array(ratio_arr)
        heading_angle = np.argmin(np.abs(ratio_arr - 1))
        min_val = laser_scan[heading_angle]

        heading_angle = heading_angle + (angle_offset / 2) - 90

        return (int(heading_angle * 2), min_val)

    def find_nearest(self, ls, value):
        for i in range(len(ls)):
            if value < ls[i]:
                if i == 0:
                    return 0
                return i - 1
        return len(ls) - 1
        # arr = np.array(arr)

        # # Retrieve the index.
        # index = (np.abs(arr-value)).argmin()
        # # print(index)
        # return index

    def obtain_gain(self):
        gains = []

        for i in range(len(self.actual_dist)):
            gains.append(self.actual_dist[i] / self.measured_ratio[i])
        # gains = .69

        return gains

    def scale_gains(self, gains, measurement):
        # if(not(measurement)):
        #     return 0
        if not (measurement):
            return 0
        index = self.find_nearest(self.measured_ratio, measurement)
        # print(gains[index])
        # print(gains[index], gains[index+1])
        # print(gains[index] * self.measured_ratio[index])
        if index < len(self.actual_dist) - 1:

            cur_act_val = self.actual_dist[index]
            next_act_val = self.actual_dist[index + 1]
            cur_val = self.measured_ratio[index]
            next_val = self.measured_ratio[index + 1]

            denom = next_val - cur_val
            # print(measurement, cur_val, measurement - cur_val)
            estimation = (measurement - cur_val) / denom
            act_dif_est = estimation * (next_act_val - cur_act_val)
            # print(act_dif_est)
            # print(estimation)
            # print(gains)
            k = gains[index] * self.measured_ratio[index] + act_dif_est
        else:
            k = gains[index] * measurement

        # print(k)

        return k


def main(args=None):
    rclpy.init(args=args)

    ag_dist_orien = AgDistOrienCalc()

    rclpy.spin(ag_dist_orien)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_dist_orien.destroy_node()
    rclpy.shutdown()
