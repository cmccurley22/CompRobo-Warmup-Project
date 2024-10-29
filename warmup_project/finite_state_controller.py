import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from math import pi

class DriveSquareAndStop(Node):
    def __init__(self):
        super().__init__("drive_square_with_estop")
        
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # boolean representing if the robot is turning
        self.turning = 0

        # control dist and time for square
        self.side_length = 1
        self.side_time = 5
        self.turn_time = 2

        # track time passed
        self.segment_start_t = None

        # e-stop event
        self.e_stop = Event()
        self.create_subscription(Bump, "bump", self.estop, 10)

        # thread for run loop
        self.run_loop_thread = Thread(target = self.run_loop)
        self.run_loop_thread.start()


    def estop(self, msg):
        # check if any of the bumpers are hit
        bumper_active = (msg.left_front == 1 or \
            msg.left_side == 1 or \
            msg.right_front == 1 or \
            msg.right_side == 1)
        
        # stop if bumper hit
        if bumper_active:
            self.e_stop.set()

            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
    
    def run_loop(self):
        while True:
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
            elif not self.e_stop.is_set():
                if self.turning:
                    # robot is turning in place
                    print("robot turning")
                    twist_msg.angular.z = (.5 * pi) / segment_total_t
                    twist_msg.linear.x = 0.0
                else:
                    # robot is driving forward
                    print("robot driving straight")
                    twist_msg.linear.x = self.side_length / segment_total_t
                    twist_msg.angular.z = 0.0
            else:
                # just for debugging, estopped
                print("estop!")
            
            self.vel_pub.publish(twist_msg)
        


def main(args=None):
    rclpy.init(args = args)
    node = DriveSquareAndStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
