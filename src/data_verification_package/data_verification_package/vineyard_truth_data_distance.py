import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int16


import numpy as np
import re


class AgTruthDistance(Node):

    def __init__(self):
        super().__init__("ag_vineyard_distance")

        self.ser = serial.Serial(
            port="/dev/ttyACM0",
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )

        self.left_distance_pub = self.create_publisher(
            Int16, "/ag/vineyard/left_distance", 10
        )
        self.right_distance_pub = self.create_publisher(
            Int16, "/ag/vineyard/right_distance", 10
        )
        timer_period = 0.1
        self.distance_timer = self.create_timer(timer_period, self.timer_callback)

        self.distance_prev = [0, 0]

    def get_serial_distances(self) -> list:
        distance_string = str(self.ser.readline())
        pattern = r"\b\d+\b"

        distances = re.findall(pattern, distance_string)
        if len(distances) == 0:
            return [self.distance_prev[0], self.distance_prev[1]]
        distances = [int(dist) for dist in distances]

        return distances

    def timer_callback(self) -> None:
        distance_one = Int16()
        distance_two = Int16()

        distances = self.get_serial_distances()
        self.distance_prev = distances

        distance_one.data = distances[0]
        distance_two.data = distances[1]

        print("===============================================")
        print(
            "Distance One: %d\nDistance Two: %d"
            % (distance_one.data, distance_two.data)
        )
        print("===============================================\n")
        self.left_distance_pub.publish(distance_one)
        self.right_distance_pub.publish(distance_two)


def main(args=None):
    rclpy.init(args=args)

    ag_vineyard_distance = AgTruthDistance()

    rclpy.spin(ag_vineyard_distance)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_vineyard_distance.destroy_node()
    rclpy.shutdown()
