import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import json
import math

class Driver(Node):
    def __init__(self):
        super().__init__('ag_percept_driver')

        self.pose_sub = self.create_subscription(
            String,
            '/ag/percept/dist_and_angle',
            self.driver_callback,
            10
        )

        self.control_pub = self.create_publisher(Twist, 'rov/percept/control', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.control_callback)
        
        self.lat_err = 0
        self.ang_err = 0

        self.u_lx = 0
        self.u_az = 0

        self.kp_theta = 0.1
        self.kp_lat = 0.1

    def driver_callback(self, msg):
        pos_dict = json.loads(msg.data)
        self.lat_err = 0 - (pos_dict['l_dist']-pos_dict['r_dist'])
        self.ang_err = 0 - pos_dict['angle']

        self.u_lx = .1
        self.u_az = self.kp_theta*self.ang_err + self.kp_lat*self.lat_err

    def control_callback(self):
        msg = Twist()
        msg.linear.x = float(self.u_lx)
        msg.angular.z = float(self.u_az)

        self.control_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    ag_percept_driver = Driver()

    rclpy.spin(ag_percept_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_percept_driver.destroy_node()
    rclpy.shutdown()