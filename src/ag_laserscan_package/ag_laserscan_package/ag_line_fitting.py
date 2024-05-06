import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from ag_custom_message.msg import LidarData

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from scipy.optimize import curve_fit
from sklearn.cluster import KMeans
import math

class AgVineyardRowLineFitting(Node):

    def __init__(self):
        super().__init__('ag_line_fitting')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.filter_raw_scan,
            qos_profile=qos_profile,)
        
        self.cross_track_err_pub = self.create_publisher(Float32, "/ag/percept/crosstrack_err", 10)
        self.heading_pub = self.create_publisher(Float32, "/ag/percept/heading_err", 10)
        self.max_front = self.create_publisher(Float32, "/ag/percept/max_front", 10)
        
        self.max_scan_range = 5

        self.cross_track_err = 0
        self.ranges = []

        self.dist_arr_left = []
        self.dist_arr_right = []

        self.angle_arr_left = []
        self.angle_arr_right = []
  
    def is_within_range(self, value) -> bool:
        return value <= self.max_scan_range
    
    def is_in_area_of_interest(self, degree, area_of_interest) -> bool:
        return degree in range(area_of_interest[0],area_of_interest[1])
    
    def is_valid_line(self, points) -> bool:
        return len(points[0]) > 1 and len(points[1]) > 1
    
    def get_x_from_polar_coords(self, degree, distance) -> float:
        return distance*math.cos(math.radians(degree))
    
    def get_y_from_polar_coords(self, degree, distance) -> float:
        return distance*math.sin(math.radians(degree))
    
    def get_polar_coordinates(self, scan, area_of_interest) -> dict:
        polar_coords = {}
        for degree, distance in enumerate(scan):
            if not(math.isinf(distance)) and self.is_in_area_of_interest(degree, area_of_interest):
                polar_coords[degree] = distance
        return polar_coords
    
    def get_cartesian_coords(self, polar_coords) -> list:
        x = np.array([self.get_x_from_polar_coords(degree, distance) for degree, distance in polar_coords.items()])
        y = np.array([self.get_y_from_polar_coords(degree, distance) for degree, distance in polar_coords.items()])     
        return [x,y]
    
    def get_line_parameters(self, cartesian_coords) -> tuple:
        x = cartesian_coords[0]
        # x = np.array([1,3])
        y = cartesian_coords[1]
        # y = np.array([2,3])

        def line_objective(x, a, b):
            return a * x + b
        
        popt, _ = curve_fit(line_objective, x, y)
        m, b = popt
        return (m, b)
    
    def get_standard_line(self, m, b) -> tuple:
        A = -m
        B = 1
        C = -b

        return (A, B, C)
    
    def get_shortest_distance(self, line, point) -> float:
        A, B, C = self.get_standard_line(line[0], line[1])

        numerator = ((A*point[0]) + (B*point[1]) + C)
        denominator = math.sqrt(A**2 + B**2)

        return numerator/denominator
    
    # Need to implement this function further (Automatically detect the vineyard rows using KMeans).
    def get_laserscan_clusters(self, x, y):
        X = np.column_stack((x,y))

        clusters = KMeans(n_clusters=2)
        clusters.fit(X)

        labels = clusters.labels_
    
    def moving_avg_filter(self, new_value, values, window_size=10):
        avg = 0
        if(len(values) >= 10):
            values.pop(0)
            avg = sum(values)/window_size
        values.append(new_value)
        return avg

    def determine_range(self, scan, area_of_interest) -> int:
        polar_coords = self.get_polar_coordinates(scan, area_of_interest)
        cartesian_coords = self.get_cartesian_coords(polar_coords)

        # if(area_of_interest[0] >= 0 and area_of_interest[1] <= 179):
        #     cartesian_coords[1] = [-1*y for y in cartesian_coords[1]]
        #     print('negating y')
        
        # if(area_of_interest[0] >= 89 and area_of_interest[1] <= 269):
        #     cartesian_coords[0] = [-1*x for x in cartesian_coords[0]]
        #     print('negating x')

        if not(self.is_valid_line(cartesian_coords)):
            print("Unable to produce valid line.")
            return [None, None]
        
        # slope, intercept = np.polyfit(cartesian_coords[1], cartesian_coords[0], deg=1)
        # print("X: %s\nY: %s" % (str(cartesian_coords[0]), str(cartesian_coords[1])))
        # print("Slope: %f" % (slope))ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --remap cloud_in:=/quanergy/points


        line_parameters = self.get_line_parameters(cartesian_coords)
        distance = abs(self.get_shortest_distance(line_parameters, (0,0)))
        heading = -math.degrees(math.atan(line_parameters[0]))
        distance_to_point = math.sqrt(cartesian_coords[0][0]**2+cartesian_coords[1][0]**2)

        if distance < 1:
            return [None, None]

        print('-------------------------------------------------------')
        print("Scan Range: %s" % (str(area_of_interest)))
        print("Slope: %f\nDistance: %f\nDegrees: %f\nDistance to point: %f" % (line_parameters[0], distance, heading, distance_to_point))
        print('-------------------------------------------------------')

        return (distance, heading)

    def filter_raw_scan(self, msg):
        lidar_data_msg = LidarData()
        max_front_msg = Float32()
        scan = msg.ranges
        right_scan, right_heading = self.determine_range(scan, [89, 159])
        if(right_scan != None):
            right_scan = self.moving_avg_filter(right_scan, self.dist_arr_right)
            right_heading = self.moving_avg_filter(right_heading, self.angle_arr_right)
        left_scan, left_heading = self.determine_range(scan, [199,269])
        if(left_scan!=None):
            left_scan = self.moving_avg_filter(left_scan, self.dist_arr_left)
            left_heading = self.moving_avg_filter(left_heading, self.angle_arr_left)

        max_front_msg.data = scan[179]
        print("Max dist: %f" % max_front_msg.data)
        self.max_front.publish(max_front_msg)

        if(left_scan != None and right_scan != None):
            headings = [left_heading, right_heading]

            if(abs(left_heading) - abs(right_heading) > 10):
                select = np.argmin([abs(left_heading), abs(right_heading)])
                print(select)
            else:
                headings = [left_heading+right_heading/2,left_heading+right_heading/2]
                select = 0

            print(select)
            heading = headings[select]
            self.cross_track_err = left_scan - right_scan
            cross_track_msg = Float32()
            heading_msg = Float32()
            cross_track_msg.data = float(self.cross_track_err)
            heading_msg.data = float(heading)

            print('-------------------------------------------------------')
            print('Crosstrack Error: %f' % self.cross_track_err)
            print('Heading: %f' % heading)
            print('-------------------------------------------------------')
            # print("Left return code: %d" % (left_scan))
            print("Return code: %d" % (left_scan))
            self.cross_track_err_pub.publish(cross_track_msg)
            self.heading_pub.publish(heading_msg)
        

def main(args=None):
    rclpy.init(args=args)

    ag_line_fitting = AgVineyardRowLineFitting()

    rclpy.spin(ag_line_fitting)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_line_fitting.destroy_node()
    rclpy.shutdown()