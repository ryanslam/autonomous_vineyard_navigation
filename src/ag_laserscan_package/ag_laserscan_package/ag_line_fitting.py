import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

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
        
        self.max_scan_range = 5

        self.ranges = []
  
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
            return -1
        
        # slope, intercept = np.polyfit(cartesian_coords[1], cartesian_coords[0], deg=1)
        print("X: %s\nY: %s" % (str(cartesian_coords[0]), str(cartesian_coords[1])))
        # print("Slope: %f" % (slope))

        line_parameters = self.get_line_parameters(cartesian_coords)
        distance = self.get_shortest_distance(line_parameters, (0,0))
        heading = -math.degrees(math.atan(line_parameters[0]))
        distance_to_point = math.sqrt(cartesian_coords[0][0]**2+cartesian_coords[1][0]**2)
        cross_track_err = distance_to_point

        print('-------------------------------------------------------')
        print("Scan Range: %s" % (str(area_of_interest)))
        print("Slope: %f\nDistance: %f\nDegrees: %f\nCross Track Err: %f" % (line_parameters[0], distance, heading, cross_track_err))
        print('-------------------------------------------------------')

        return 0

    def filter_raw_scan(self, msg):
        scan = msg.ranges
        # left_scan = self.determine_range(scan, [89, 179])
        left_scan = self.determine_range(scan, [179,329])
        # print("Left return code: %d" % (left_scan))
        print("Left return code: %d" % (left_scan))

def main(args=None):
    rclpy.init(args=args)

    ag_line_fitting = AgVineyardRowLineFitting()

    rclpy.spin(ag_line_fitting)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_line_fitting.destroy_node()
    rclpy.shutdown()