import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from ocp_robocon2023.casadi_solver_elephant import CasadiNMPC

class TestSubscriber(Node):

    def __init__(self):
        super().__init__('test_wait_timer')
        self.subscriptions_ = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            100
        )
        self.subscriptions_

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    test_subscriber = TestSubscriber()

    rclpy.spin(test_subscriber)

    test_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()