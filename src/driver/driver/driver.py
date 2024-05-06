import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from ag_custom_message.msg import LidarData

from geometry_msgs.msg  import Twist
from geometry_msgs.msg import Pose2D

import math

class Driver(Node):

    def __init__(self):
        super().__init__('driver')
        # self.pose_sub = self.create_subscription(
        #     LidarData,
        #     '/ag/percept/lidar_data',
        #     self.driver_callback,
        #     10)
        
        self.cross_track_sub = self.create_subscription(
            Float32,
            '/ag/percept/crosstrack_err',
            self.cross_track_callback,
            10)

        self.cross_track_sub = self.create_subscription(
            Float32,
            '/ag/percept/heading_err',
            self.heading_callback,
            10)

        self.angular_conf_sub = self.create_subscription(
            Float32,
            "/ag/percept/lidar_confidence",
            self.set_drive_flag,
            10)

        self.control_pub = self.create_publisher(Twist, '/r4/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.driver_callback)
        self.i = 0
        
        self.lat_err = 0
        self.ang_err = 0

        self.u_lx = 0
        self.uaz = 0

        self.kp_theta = 0.01
        self.kp_lat = 0.4

        self.drive_lock = 0
        self.cross_track_err = 0
        self.heading = 0

    def cross_track_callback(self, msg):
        self.cross_track_err = msg.data
    
    def heading_callback(self, msg):
        self.heading = msg.data
        

    def driver_callback(self):
        msg_twist = Twist()
        self.lat_err = self.cross_track_err
        #print(self.lat_err)
        self.ang_err = 0 - self.heading

        if(abs(self.lat_err)<1):
            self.u_lx = .3
        else:
            self.u_lx = 0
        self.uaz = self.kp_theta*self.ang_err + self.kp_lat*self.lat_err
        print('angular: ' + str(self.kp_theta*self.ang_err))
        print('lateral: ' + str(self.kp_lat*self.lat_err))
        ls = [self.ang_err, self.kp_theta*self.ang_err, self.kp_lat*self.lat_err]
        # print(ls)
        if(self.uaz > .3):
            self.uaz = .3
        elif(self.uaz < -.3):
            self.uaz = -.3

        msg_twist.linear.x = float(self.u_lx)
        msg_twist.angular.z = float(self.uaz)

        self.control_pub.publish(msg_twist)


    def control_callback(self):
        msg = Twist()
        if(self.drive_lock):
            msg.linear.x = 0
            msg.linear.z = 0
        msg.linear.x = float(self.u_lx)
        msg.angular.z = float(self.uaz)
        
        self.control_pub.publish(msg)

    def set_drive_flag(self, msg):
        if(msg.data == 0.0):
            self.drive_lock = 1
        else:
            self.drive_lock = 0

def main(args=None):
    rclpy.init(args=args)

    driver = Driver()

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()