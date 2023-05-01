import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from .library.rabbit_omni import RabbitModel



class PS4Remote(Node):

    def __init__(self):

        super().__init__('PS4_Node')

        self.rabbit_model = RabbitModel()

        self.subscribe_joy = self.subscriptions(
            Joy, "joy", self.subscribe_callback, 10
        )

        self.publisher_twist = self.create_publisher(
            Joy, "cmd_vel", 10
        )

        # self.twist_timer = self.create_timer(0.05, self.)


    def subscribe_callback(self, joy_msg):
        joy_msg = Joy()
        joy_msg.axes[0] = joy_msg.axes[0]
        joy_msg.axes[1] = joy_msg.axes[1]
        joy_msg.axes[2] = joy_msg.axes[2]
        joy_msg.axes[3] = joy_msg.axes[3]

        joy_msg.axes[4] = joy_msg.axes[4]
        joy_msg.axes[5] = joy_msg.axes[5]
        joy_msg.axes[6] = joy_msg.axes[6]
        joy_msg.axes[7] = joy_msg.axes[7]

        self.get_logger().info("Joy stick value ", joy_msg)



def main(args=None):

    rclpy.init(args=args)

    node = PS4Remote()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

