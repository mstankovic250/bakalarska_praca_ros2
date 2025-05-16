#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

# Definícia klávesových vstupov
KEY_MAPPING = {
    'w': (1.0, 0.0),  # Dopredu
    's': (-1.0, 0.0),  # Dozadu
    'a': (0.0, 1.0),   # Doľava
    'd': (0.0, -1.0),  # Doprava
    ' ': (0.0, 0.0),   # Zastaviť
}

class MoveBall(Node):
    def __init__(self):
        super().__init__('move_ball')
        self.publisher = self.create_publisher(Twist, '/ball_cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.get_logger().info("Používajte klávesy W (dopredu), S (dozadu), A (doľava), D (doprava), medzerník (zastaviť).")
        while rclpy.ok():
            key = self.get_key()
            if key in KEY_MAPPING:
                linear, angular = KEY_MAPPING[key]
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                self.publisher.publish(twist)
                self.get_logger().info(f"Pohyb loptičky: linear={linear}, angular={angular}")
            elif key == '\x03': 
                break

def main(args=None):
    rclpy.init(args=args)
    move_ball = MoveBall()
    move_ball.run()
    move_ball.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
