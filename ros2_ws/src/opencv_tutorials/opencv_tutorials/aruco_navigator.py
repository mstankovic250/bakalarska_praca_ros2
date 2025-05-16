import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import math


class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')

        # ROS 2 Subscribers & Publishers
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # TF Listener pre získanie polohy robota
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Nastavenie ArUco detektora
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Kamera kalibrácia
        self.camera_matrix = np.array([[554.38, 0, 320], [0, 554.38, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros((4, 1))

    def image_callback(self, msg):
        """ Spracuje obraz a detekuje ArUco marker. """
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Konverzia na čiernobiely obraz
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detekcia ArUco markerov
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                self.get_logger().info(f"Detekovaný ArUco marker ID: {marker_id}")

                # Vypočítanie pozície markera
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.15, self.camera_matrix,
                                                                      self.dist_coeffs)
                world_x, world_y = self.convert_to_world_coordinates(tvecs[i])

                if world_x is not None and world_y is not None:
                    self.navigate_to_goal(world_x, world_y)

            # Vykreslenie detekovaných markerov
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    def convert_to_world_coordinates(self, tvec):
        """ Konvertuje pozíciu markera z kamery do svetových súradníc. """
        try:
            transform = self.tf_buffer.lookup_transform('world', 'camera', rclpy.time.Time())

            # Kamerová pozícia
            camera_pos = np.array([transform.transform.translation.x,
                                   transform.transform.translation.y,
                                   transform.transform.translation.z])

            # Transformácia markerovej pozície
            world_x = camera_pos[0] + tvec[0][0]
            world_y = camera_pos[1] + tvec[0][1]

            self.get_logger().info(f"Transformovaný marker: X={world_x:.2f}, Y={world_y:.2f}")
            return world_x, world_y

        except Exception as e:
            self.get_logger().warning(f"Transformácia zlyhala: {e}")
            return None, None

    def navigate_to_goal(self, goal_x, goal_y):
        """ Naviguje robota na zistenú pozíciu markera. """
        try:
            transform = self.tf_buffer.lookup_transform('world', 'telo', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            delta_x = goal_x - robot_x
            delta_y = goal_y - robot_y
            distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
            target_angle = math.atan2(delta_y, delta_x)

            quat = transform.transform.rotation
            _, _, robot_yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

            twist = Twist()

            while abs(target_angle - robot_yaw) > 0.05:
                error = target_angle - robot_yaw
                twist.linear.x = 0.0
                twist.angular.z = min(0.5, max(-0.5, error * 2.0))
                self.cmd_pub.publish(twist)

                rclpy.spin_once(self)
                transform = self.tf_buffer.lookup_transform('world', 'telo', rclpy.time.Time())
                quat = transform.transform.rotation
                _, _, robot_yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self)

            while distance > 0.1:
                transform = self.tf_buffer.lookup_transform('world', 'telo', rclpy.time.Time())
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y
                delta_x = goal_x - robot_x
                delta_y = goal_y - robot_y
                distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

                twist.linear.x = 0.3
                self.cmd_pub.publish(twist)

                rclpy.spin_once(self)

            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)

            self.get_logger().info(f"Robot dosiahol cieľ na {goal_x}, {goal_y}")

        except Exception as e:
            self.get_logger().warning(f"Navigácia zlyhala: {e}")

    def quaternion_to_euler(self, x, y, z, w):
        """ Konverzia quaternionu na Eulerove uhly. """
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 0.0, 0.0, yaw


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
