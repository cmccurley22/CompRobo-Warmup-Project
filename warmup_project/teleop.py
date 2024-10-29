import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# bind keys to their corresponding velocities
key_bindings = {
    "w": (1.0, 0.0), # forward
    "a": (0.0, 1.0), # spin left
    "s": (-1.0, 0.0),  #backward
    "d": (0.0, -1.0), # spin right
}


class TeleOpNode(Node):
    def __init__(self):
        super().__init__("teleop")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_timer(0.1, self.run_loop)

        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self):
        '''get key being pressed from keyboard'''
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run_loop(self):
        '''loop checking keypresses and updating robot velocity'''
        while self.key_pressed != '\x03':
            self.get_key()

            if self.key_pressed in key_bindings:
                # w, a, s, or d being pressed - get corresponding velocity
                vels = key_bindings[self.key_pressed]

                # set neato velocity based on keypress
                twist_msg = Twist()
                twist_msg.linear.x = vels[0]
                twist_msg.angular.z = vels[1]

                self.vel_pub.publish(twist_msg)


def main(args = None):
    rclpy.init(args = args)
    node = TeleOpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

