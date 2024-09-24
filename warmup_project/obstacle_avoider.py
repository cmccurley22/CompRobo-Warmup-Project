import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians, atan


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", \
            self.follow_wall, 10)

        self.create_timer(0.1, self.run_loop)

        self.scan_angles = (270 - 45, 270 + 45)

        self.wall_dist = 0
        self.wall_angle = 0

        self.goal_dist = 1
        self.goal_angle = 0

    def get_wall_dist(self, ranges, a1, a2):
        if ranges[a1] and ranges[a2]:
            x1 = self.ranges[a1] * cos(radians(270 - a1))
            y1 = -self.ranges[a1] * sin(radians(270 - a1))
            x2 = self.ranges[a2] * cos(radians(a2 - 270))
            y2 = self.ranges[a2] * sin(radians(a2 - 270))

            m = (y2 - y1) / (x2 - x1)
            b = -m * x1  + y1

            self.wall_dist = abs(b) / sqrt(1 + m * m)

            if m != 0: # need to make sure denominator isn't 0
                self.wall_angle = atan(1 / m)
    
    def follow_wall(self, scan_msg):
        k_p = 10 # proportional control constant

        self.get_wall_dist(scan_msg.ranges, 225, 315)

        err_dist = self.wall_dist - self.goal_dist
        err_angle = self.wall_angle - self.goal_angle
        

        


def main(args = None):
    rclpy.init(args = args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

