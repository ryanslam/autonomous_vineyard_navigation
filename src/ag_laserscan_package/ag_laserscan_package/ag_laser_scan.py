import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
import math

from cv_bridge import CvBridge, CvBridgeError

# Discretization Size
disc_size = .08
# Discretization Factor
disc_factor = 1/disc_size
# Max Lidar Range
max_lidar_range = 10
# Create Image Size Using Range and Discretization Factor
image_size = int(max_lidar_range*2*disc_factor)

class AgLaserScan(Node):

    def __init__(self):
        super().__init__('ag_laser_scan')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.listener_callback,
            qos_profile=qos_profile,)
        
        self.laserscan_pub = self.create_publisher(LaserScan, '/ag/scan/ranges', 10)
        self.pub = self.create_publisher(Image, "/scan_to_image", 10)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning

    def calculate_range(self, rho, theta, c=1):
        return rho + (c*math.sin(theta))

    def listener_callback(self, scan):
		# Store maxAngle of lidar
        maxAngle = scan.angle_max
		# Store minAngle of lidar
        minAngle = scan.angle_min
		# Store angleInc of lidar
        angleInc = scan.angle_increment
		# Store maxLength in lidar distances
        maxLength = scan.range_max
		# Store array of ranges
        ranges = scan.ranges
		# Calculate the number of points in array of ranges
        num_pts = len(ranges)
		# Create Array for extracting X,Y points of each data point
        xy_scan = np.zeros((num_pts,2))
		# Create 3 Channel Blank Image
        blank_image = np.zeros((image_size,image_size,3),dtype=np.uint8)
		# Loop through all points converting distance and angle to X,Y point
        for i in range(num_pts):
			# # Check that distance is not longer than it should be
            if (ranges[i] > max_lidar_range) or (ranges[i] < 0) or (math.isnan(ranges[i])):
                pass
            else:
                # Calculate angle of point and calculate X,Y position
                print(ranges[270])
                angle = minAngle + float(i)*angleInc
                xy_scan[i][0] = float(ranges[i]*math.cos(angle))
                xy_scan[i][1] = float(ranges[i]*math.sin(angle))

		# Loop through all points plot in blank_image
        for i in range(num_pts):
            pt_x = xy_scan[i,0]
            pt_y = xy_scan[i,1]
            if (pt_x < max_lidar_range) or (pt_x > -1 * (max_lidar_range-disc_size)) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range-disc_size)):
                pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                if (pix_x > image_size) or (pix_y > image_size):
                    print('error')
                else:
                    n = 5
                    blank_image[pix_y,pix_x] = [0,0,255]
                    # print()
                    # if(pix_y > image_size//2):
                    #     blank_image[pix_y-n:pix_y+n,pix_x-n:pix_x+n] = [0,0,255]

        blank_image = cv2.rotate(blank_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

		# Convert CV2 Image to ROS Message
        img = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
		# Publish image
        self.pub.publish(img)
        # for i in range(360):
        #     if msg.ranges[i] > 2 or msg.ranges[i] < 1:
        #         msg.ranges[i] = 0
        # # self.valid_ranges = [val if val <=3 or val >=1 else 0 for val in msg.ranges]
        # self.laserscan_pub.publish(msg)
        # print('deg\t\n0: %f\t\n90: %f\t\n180: %f\n' % (msg.ranges[0], msg.ranges[89], msg.ranges[179]))
        # f = open('/home/toughbook/Desktop/test.txt', "a")
        # f.write(str(msg.ranges))
        # f.close()
def main(args=None):
    rclpy.init(args=args)

    ag_laser_scan = AgLaserScan()

    rclpy.spin(ag_laser_scan)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ag_laser_scan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
