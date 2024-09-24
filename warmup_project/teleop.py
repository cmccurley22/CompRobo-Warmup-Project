import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


key_bindings = {
    "w": (1.0, 0.0),
    "a": (0.0, 1.0),
    "s": (-1.0, 0.0),
    "d": (0.0, -1.0),
}


class TeleOpNode(Node):
    def __init__(self):
        super().__init__("teleop")

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_timer(0.1, self.run_loop)

        self.key_pressed = None
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key_pressed = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run_loop(self):
        while self.key_pressed != '\x03':
            self.get_key()

            if self.key_pressed in key_bindings:
                vels = key_bindings[self.key_pressed]

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

