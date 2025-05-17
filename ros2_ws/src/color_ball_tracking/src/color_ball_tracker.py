#!/usr/bin/env python3

# ---------- EXPERIMENT 1 ----------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorBallTracker(Node):
    def __init__(self):
        super().__init__('color_ball_tracker')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.stop_threshold = 20000  
        self.image_width = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            mask = cv2.inRange(hsv_image, lower_red, upper_red)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            twist = Twist()

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if self.image_width is None:
                    self.image_width = cv_image.shape[1]

                if area >= self.stop_threshold:
                    self.get_logger().info("Loptička je blízko, zastavujem.")
                    twist.linear.x = 0.0
                else:
                    M = cv2.moments(largest_contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])

                        twist.linear.x = 0.2
                        twist.angular.z = -0.01 * (cx - self.image_width / 2)

            else:
                self.get_logger().info("Loptička zmizla, zastavujem.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Chyba pri spracovaní obrazu: {e}')

def main(args=None):
    rclpy.init(args=args)
    tracker = ColorBallTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
