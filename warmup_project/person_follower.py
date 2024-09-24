import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians, atan, floor, isinf


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", \
            self.run_loop, 10)

        # self.create_timer(0.1, self.run_loop)

        self.scan_angles = (270 - 45, 270 + 45)

        # difference of about 1ft means new object
        self.diff_threshold = .33

        # position of person
        self.person_angle = 0
        self.person_dist = 0

    def get_scan(self, scan_msg):
        '''read lidar data'''
        self.ranges = scan_msg.ranges
    
    def find_person(self):
        last_dist = 0
        obj_start_found = False
        self.obj_found = False

        objs = []

        for angle in range(-45, 45):
            theta = angle
            if theta < 0:
                theta += 360

            curr_dist = self.ranges[theta]

            if curr_dist != 0:
                if not obj_start_found:
                    if (last_dist - curr_dist > self.diff_threshold):
                        # start of object found!
                        self.obj_found = True
                        
                        obj_start_found = True
                        print("aaaaa")
                        objs.append([angle, 0])
                    
                else:
                    # checking for end of object
                    if (curr_dist - last_dist > self.diff_threshold):
                        obj_start_found = False
                        print(objs)
                        objs[-1][1] = angle


            last_dist = curr_dist

        # find the largest object based on angle ranges
        # thats the person :)
        if self.obj_found:
            print("object found!")
            person = max(objs, key = lambda x: abs(x[1] - x[0]))
            self.person_angle = floor((person[1] - person[0]) / 2)
            if self.person_angle < 0:
                self.person_angle += 360
            
            print(f"checking angle: {self.person_angle}")
            self.person_dist = self.ranges[self.person_angle]
            print(f"person dist: {self.person_dist}")

            if isinf(self.ranges[self.person_angle]):
                self.person_angle = person[0]
            while isinf(self.ranges[self.person_angle]):
                # if center of range not in scan, then go through
                # range to find a valid value to move toward
                self.person_angle += 1

    def vel_controller(self):
        k_p_ang = 0.2
        k_p_lin = 0.1

        lin_vel = self.person_dist * k_p_lin
        ang_vel = radians(self.person_angle) * k_p_ang

        # TODO: velocity limits
        max_lin = 0
        lin_vel = max(max_lin, lin_vel)

        if isinf(lin_vel):
            lin_vel = 0.0

        twist_msg = Twist()

        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel

        print(f"linear vel: ${lin_vel}")
        print(f"angular vel: ${ang_vel}")

        self.vel_pub.publish(twist_msg)
    
    def run_loop(self, scan_msg):
        self.ranges = scan_msg.ranges
        # self.get_scan()
        self.find_person()
        if self.obj_found:
            self.vel_controller()


def main(args = None):
    rclpy.init(args = args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

