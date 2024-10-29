import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians, atan, floor, isinf


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower")

        # publish to cmd_vel, subscribe to scan
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", \
            self.run_loop, 10)

        self.scan_angles = (270 - 45, 270 + 45)

        # difference threshold to decide if new vs same object
        self.diff_threshold = .33

        # position of person
        self.person_angle = 0
        self.person_dist = 0
    
    def find_person(self):
        '''locate nearest object (person) and save its location'''
        last_dist = 0

        # track if looking for start or end of object
        obj_start_found = False

        # has an object been found yet
        self.obj_found = False

        # list of found objects
        objs = []

        for angle in range(-45, 45):
            theta = angle

            # restrict angle to 0-360deg range
            if theta < 0:
                theta += 360

            curr_dist = self.ranges[theta]

            if curr_dist != 0:
                if not obj_start_found:
                    # checking for start of object
                    if (last_dist - curr_dist > self.diff_threshold):
                        # start of object found!
                        self.obj_found = True
                        obj_start_found = True
                        print("start of object found")

                        # append to object list with start angle
                        objs.append([angle, 0])
                    
                else:
                    # checking for end of object
                    if (curr_dist - last_dist > self.diff_threshold):
                        # end of obj found - go back to looking for new objs
                        obj_start_found = False
                        print(objs)

                        # update last object of object list with end angle
                        objs[-1][1] = angle


            last_dist = curr_dist

        # find the "largest" object based on angle ranges
        # thats the person :)
        if self.obj_found:
            print("object found!")

            # get object with greatest difference between start and end angles
            person = max(objs, key = lambda x: abs(x[1] - x[0]))

            # consider the object to be at the center of its angle range
            self.person_angle = floor((person[1] - person[0]) / 2)

            # restrict angle to 0-360deg range
            if self.person_angle < 0:
                self.person_angle += 360
            
            # get distance to object at that angle based on laser scan
            print(f"checking angle: {self.person_angle}")
            self.person_dist = self.ranges[self.person_angle]
            print(f"person dist: {self.person_dist}")

            # if center of range not in scan, then go through
            # range to find a valid value to move toward
            if isinf(self.ranges[self.person_angle]):
                self.person_angle = person[0]
            while isinf(self.ranges[self.person_angle]):
                self.person_angle += 1

    def vel_controller(self):
        '''control robot movement based on person location'''

        # proportional control constants k_p
        k_p_ang = 0.2
        k_p_lin = 0.1

        # velocity = error * k_p
        lin_vel = self.person_dist * k_p_lin
        ang_vel = radians(self.person_angle) * k_p_ang

        # if accidental infinite velocity, reset to 0
        if isinf(lin_vel):
            lin_vel = 0.0

        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel

        # print for debugging
        print(f"linear vel: ${lin_vel}")
        print(f"angular vel: ${ang_vel}")

        self.vel_pub.publish(twist_msg)
    
    def run_loop(self, scan_msg):
        # update lidar scan
        self.ranges = scan_msg.ranges

        # find nearest person
        self.find_person()

        # if a person is found, move towards it
        if self.obj_found:
            self.vel_controller()


def main(args = None):
    rclpy.init(args = args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

