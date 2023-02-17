import rclpy
from rclpy.node import Node
from model_kinematic.kinematic import *

# === ROS2 message ===
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class ros_node(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.kinematic = kinematic()
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.timer = 0.01

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel',10)

        self.twist_timer = self.create_timer(self.timer, self.twist_callback)
    
    def joy_callback(self, joy_msg):
        self.vx = joy_msg.axes[1]
        self.vy = joy_msg.axes[0]
        self.omega = joy_msg.axes[3]

    def twist_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.vx
        twist_msg.linear.y = -self.vy
        twist_msg.angular.z = self.omega
        data = np.array([self.vx, -self.vy, self.omega])
        print(data)
        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = ros_node()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()


