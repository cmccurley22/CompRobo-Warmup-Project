import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi


class DriveSquareNode(Node):
    def __init__(self):
        super().__init__("drive_square")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_timer(0.1, self.run_loop)

        # boolean representing if the robot is turning
        self.turning = 0

        # control dist and time for square
        self.side_length = 1
        self.side_time = 5
        self.turn_time = 2

        # track time passed
        self.segment_start_t = None

    def run_loop(self):
        if self.segment_start_t is None:
            self.segment_start_t = self.get_clock().now()
        
        twist_msg = Twist()

        # update segement time based on turning state
        segment_total_t = self.turn_time if self.turning else self.side_time

        if self.get_clock().now() - self.segment_start_t > \
            rclpy.time.Duration(seconds = segment_total_t):
            # finished segment: reset time, toggle turning state
            self.turning = not self.turning
            self.segment_start_t = None
        else:
            if self.turning:
                # robot is turning in place
                print("state: turning")
                twist_msg.angular.z = (.5 * pi) / segment_total_t
                twist_msg.linear.x = 0.0
            else:
                # robot is driving forward
                print("state: driving forwards")
                twist_msg.linear.x = self.side_length / segment_total_t
                twist_msg.angular.z = 0.0
        
        self.vel_pub.publish(twist_msg)


def main(args = None):
    rclpy.init(args = args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

