# ---------- EXPERIMENT 3 ----------


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
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

        self.camera_matrix = np.array([[4.970870507466349295e+02, 0, 3.168387473166100108e+02],
                                       [0, 4.976011127668800782e+02, 2.342740555042151982e+02],
                                       [0, 0, 1]])
        self.dist_coeffs = np.zeros((4, 1))

        self.goal_pose_odom = None
        self.navigation_active = False
        self.clicked_pixel = None
        self.sim_started = False

        cv2.namedWindow("Camera View")
        cv2.setMouseCallback("Camera View", self.mouse_click_callback)

        self.sim_check_timer = self.create_timer(1.0, self.check_sim_time)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def mouse_click_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Kliknutý bod: x={x}, y={y}")
            self.clicked_pixel = (x, y)

    def check_sim_time(self):
        if self.get_clock().now().nanoseconds > 1e9 and not self.sim_started:
            self.sim_started = True
            self.get_logger().info("Simulácia je spustená.")
            self.sim_check_timer.cancel()

    def image_callback(self, msg):
        if not self.sim_started:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) >= 3:
            image_points = []
            world_points = []

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                marker_center = np.mean(corners[i][0], axis=0)

                world_pos = self.get_marker_world_position(marker_id)
                if world_pos is not None:
                    image_points.append(marker_center)
                    world_points.append(world_pos)

            if len(image_points) == 3:
                image_pts = np.array(image_points, dtype=np.float32)
                world_pts = np.array(world_points, dtype=np.float32)

                A = cv2.getAffineTransform(image_pts, world_pts)

                if self.clicked_pixel is not None:
                    pixel = np.array([[[self.clicked_pixel[0], self.clicked_pixel[1]]]], dtype=np.float32)
                    A = A.astype(np.float32)
                    transformed = cv2.transform(pixel, A)[0][0]

                    self.get_logger().info(
                        f"Kliknutý bod transformovaný do svetových súradníc (affine): "
                        f"X={transformed[0]:.2f}, Y={transformed[1]:.2f}"
                    )

                    self.goal_pose_odom = PoseStamped()
                    self.goal_pose_odom.header.frame_id = "odom"
                    self.goal_pose_odom.pose.position.x = transformed[0]
                    self.goal_pose_odom.pose.position.y = transformed[1]
                    self.goal_pose_odom.pose.position.z = 0.0
                    self.goal_pose_odom.pose.orientation.w = 1.0

                    self.navigation_active = True
                    self.clicked_pixel = None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    def get_marker_world_position(self, marker_id):
        # Známe pozície markerov v rovine Z=0
        poses = {
            0: [5.0, 2.0],
            1: [5.0, -2.0],
            2: [7.0, 0.0],
        }
        return poses.get(marker_id, None)

    def control_loop(self):
        if not self.navigation_active or self.goal_pose_odom is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform("odom", "telo", rclpy.time.Time(),
                                                        rclpy.duration.Duration(seconds=0.5))
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            robot_yaw = self.get_yaw_from_quaternion(transform.transform.rotation)

            goal_x = self.goal_pose_odom.pose.position.x
            goal_y = self.goal_pose_odom.pose.position.y

            dx = goal_x - robot_x
            dy = goal_y - robot_y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx) - robot_yaw
            target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))

            twist = Twist()
            if abs(target_angle) > 0.3:
                twist.angular.z = 1.0 * target_angle
            elif distance > 0.2:
                twist.linear.x = 0.4
                twist.angular.z = 0.5 * target_angle
            else:
                self.get_logger().info("Dosiahnutý cieľ.")
                self.stop_robot()
                return

            self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().warn(f"Nepodarilo sa získať polohu robota: {e}")

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

