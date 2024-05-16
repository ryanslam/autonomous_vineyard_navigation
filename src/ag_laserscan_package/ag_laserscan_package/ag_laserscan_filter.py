import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import json


class AgLaserscanFilter(Node):
    def __init__(self):
        super().__init__("ag_laserscan_filter")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.laserscan_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.filter_laserscan,
            qos_profile=qos_profile,
        )

        self.filtered_laserscan_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic="ag/perception/filtered",
            callback=self.filter_laserscan,
            qos_profile=qos_profile,
        )

        self.filtered_scan_pub = self.create_publisher(
            LaserScan, "ag/perception/filtered", 10
        )
        self.average_scan_pub = self.create_publisher(
            String, "ag/perception/averaged_scan", 10
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_laserscan_average)

        self.laserscan_deg_and_dist = dict((key, []) for key in range(360))
        self.max_range = 4

    def filter_laserscan(self, msg) -> None:
        scan_ranges = msg.ranges
        for distance in range(len(scan_ranges)):
            scan_ranges[distance] = self.filter_distance_value(scan_ranges[distance])
            print(distance)

        msg.ranges = scan_ranges
        self.filtered_scan_pub.publish(msg)

    def filter_distance_value(self, distance) -> float:
        if distance >= self.max_range:
            distance = 0
        return distance

    def append_scan_value(self, msg) -> None:
        scan_ranges = msg.ranges
        for index in range[len(scan_ranges)]:
            if np.inf(scan_ranges[index]):
                continue
            self.laserscan_deg_and_dist[index].append(scan_ranges[index])

    def compute_scan_average(self) -> dict:
        averaged_laserscan = {}
        for key, value in self.laserscan_deg_and_dist.items():
            averaged_laserscan[key] = self.compute_deg_average(value)
        return averaged_laserscan

    def compute_deg_average(self, distance_list) -> float:
        if len(distance_list) == 0:
            return 0
        return sum(distance_list) / len(distance_list)

    def jsonify_string(self, dictionary) -> str:
        return json.dumps(dictionary)

    def publish_laserscan_average(self) -> None:
        msg = String()
        averaged_laserscan = self.compute_scan_average()
        msg.data = self.jsonify_string(averaged_laserscan)
        self.average_scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    filtered_laserscan = AgLaserscanFilter()

    rclpy.spin(filtered_laserscan)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    filtered_laserscan.destroy_node()
    rclpy.shutdown()
