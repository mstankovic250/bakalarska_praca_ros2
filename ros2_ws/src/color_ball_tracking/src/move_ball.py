#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

KEY_MAPPING = {
    'w': (1.0, 0.0),   # Dopredu
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
        self.linear_x = 0.0
        self.linear_y = 0.0

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
        self.get_logger().info("Ovládaj loptičku: W/S (dopredu/dozadu), A/D (doľava/doprava), SPACE (stop)")
        while rclpy.ok():
            key = self.get_key()
            if key in KEY_MAPPING:
                dx, dy = KEY_MAPPING[key]
                if key == 'w' or key == 's':
                    self.linear_x = dx
                elif key == 'a' or key == 'd':
                    self.linear_y = dy
                elif key == ' ':
                    self.linear_x = 0.0
                    self.linear_y = 0.0

                twist = Twist()
                twist.linear.x = self.linear_x
                twist.linear.y = self.linear_y
                self.publisher.publish(twist)
                self.get_logger().info(f"Pohyb loptičky: x={self.linear_x}, y={self.linear_y}")
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
