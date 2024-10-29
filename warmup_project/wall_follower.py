import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians, atan


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower")

        # publish to cmd_vel, subscribe to scan
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", \
            self.follow_wall, 10)

        # initialize wall pos
        self.wall_dist = 0
        self.wall_angle = 0

        # goal distance and angle to wall
        self.goal_dist = .5
        self.goal_angle = 0
        

    def get_wall_dist(self, ranges, a1, a2):
        '''find distance and angle of nearest wall'''
        if ranges[a1] and ranges[a2]:
            x1 = ranges[a1] * cos(radians(270 - a1))
            y1 = -ranges[a1] * sin(radians(270 - a1))
            x2 = ranges[a2] * cos(radians(a2 - 270))
            y2 = ranges[a2] * sin(radians(a2 - 270))

            m = (y2 - y1) / (x2 - x1)
            b = -m * x1  + y1

            self.wall_dist = abs(b) / sqrt(1 + m * m)

            if m != 0: # need to make sure denominator isn't 0
                self.wall_angle = atan(1 / m)
    

    def follow_wall(self, scan_msg):
        '''update robot vel based on new laser scan'''
        # get updated laser scan
        self.ranges = scan_msg.ranges 

        # proportional control constant for turning
        k_p = .6 

        # consider within range of angles 225-315
        self.get_wall_dist(self.ranges, 225, 315)

        # update robot velocity
        twist_msg = Twist()
        twist_msg.linear.x = .1
        twist_msg.angular.z = k_p * self.wall_angle

        self.vel_pub.publish(twist_msg)
        


def main(args = None):
    rclpy.init(args = args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

