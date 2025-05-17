# ---------- EXPERIMENT 2 ----------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os
from ament_index_python import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped
import traceback
from scipy.spatial.transform import Rotation as R


class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')

        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.camera_matrix = np.array([[497.087, 0, 316.838],
                                       [0, 497.601, 234.274],
                                       [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

        self.goal_pose_odom = None
        self.navigation_active = False
        self.last_detection_time = None
        self.detection_timeout = 0.5
        self.last_log_time = self.get_clock().now()

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.sim_started = False
        self.create_timer(1.0, self.check_sim_time)

    def check_sim_time(self):
        if self.get_clock().now().nanoseconds > 1e9:
            if not self.sim_started:
                self.sim_started = True
                self.get_logger().info("Simulácia je spustená.")

    def image_callback(self, msg):
        if not self.sim_started:
            return

        now = self.get_clock().now()
        should_log = (now - self.last_log_time).nanoseconds / 1e9 > 0.5

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        markerLength = 0.4
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, self.camera_matrix, self.dist_coeffs)

        if ids is not None:
            self.last_detection_time = now

            for i in range(len(ids)):
                rvec = rvecs[i]
                tvec = tvecs[i]
                marker_id = ids[i][0]

                if should_log:
                    self.get_logger().info(f"Detekovaný ArUco marker ID: {marker_id}")
                    self.get_logger().info(f"Camera tvec: {tvec}, rvec: {rvec}")

                try:
                    # Vytvorenie pozície markeru voči kamere
                    marker_pose_camera = PoseStamped()
                    marker_pose_camera.header.stamp = msg.header.stamp
                    marker_pose_camera.header.frame_id = "camera_base"

                    # Prevod tvec z OpenCV do ROS súradníc
                    marker_pose_camera.pose.position.x = float(tvec[0][2])
                    marker_pose_camera.pose.position.y = -float(tvec[0][0])
                    marker_pose_camera.pose.position.z = -float(tvec[0][1])

                    # Prevod rvec na quaternion
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    rot = R.from_matrix(rot_matrix)
                    quat = rot.as_quat()  # [x, y, z, w]

                    marker_pose_camera.pose.orientation.x = quat[0]
                    marker_pose_camera.pose.orientation.y = quat[1]
                    marker_pose_camera.pose.orientation.z = quat[2]
                    marker_pose_camera.pose.orientation.w = quat[3]

                    # transformácia z camera_base → odom
                    transform = self.tf_buffer.lookup_transform(
                        "odom", "camera_base",
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=0.5)
                    )

                    if should_log:
                        self.get_logger().info(
                            f"Robot v rámci 'odom': X={transform.transform.translation.x:.2f}, "
                            f"Y={transform.transform.translation.y:.2f}"
                        )

                    # Transformacia marker do rámca odom
                    transformed_pose = do_transform_pose_stamped(marker_pose_camera, transform)
                    self.goal_pose_odom = transformed_pose.pose
                    self.navigation_active = True

                    if should_log:
                        self.get_logger().info(
                            f"Marker v rámci 'odom': X={self.goal_pose_odom.position.x:.2f}, "
                            f"Y={self.goal_pose_odom.position.y:.2f}"
                        )

                except Exception as e:
                    if should_log:
                        self.get_logger().warn(f"Nepodarilo sa transformovať marker: {e}")
                        self.get_logger().info(traceback.format_exc())

            # Vizualizácia markerov v obraze
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

        if should_log:
            self.last_log_time = now

    def control_loop(self):
        if not self.navigation_active or self.goal_pose_odom is None:
            return

        if self.last_detection_time is not None:
            elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if elapsed > self.detection_timeout:
                self.get_logger().info("Marker stratený. Zastavujem robota.")
                self.stop_robot()
                return

        try:
            transform = self.tf_buffer.lookup_transform("odom", "telo", rclpy.time.Time(),
                                                        rclpy.duration.Duration(seconds=0.5))
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_yaw = self.get_yaw_from_quaternion(transform.transform.rotation)

            goal_x = self.goal_pose_odom.position.x
            goal_y = self.goal_pose_odom.position.y

            dx = goal_x - robot_x
            dy = goal_y - robot_y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx) - robot_yaw
            target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))

            twist = Twist()

            if abs(target_angle) > 0.3:
                twist.angular.z = 0.6 * target_angle
            elif distance > 0.2:
                twist.linear.x = 0.2
                twist.angular.z = 0.3 * target_angle
            else:
                self.get_logger().info("Dosiahnutý cieľ.")
                self.stop_robot()
                return

            self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().warn(f"Nepodarilo sa získať polohu robota: {e}")
            self.get_logger().debug(traceback.format_exc())

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.navigation_active = False
        self.goal_pose_odom = None


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




