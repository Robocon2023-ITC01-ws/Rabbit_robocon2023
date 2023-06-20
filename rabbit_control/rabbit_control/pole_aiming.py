import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String


class PoleTargeting(Node):

    def __init__(self):

        super().__init__('pole_target')

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0


        self.pole1_1 = [3.90, 0.0]
        self.pole1_2 = [4.13, 1.3]
        self.pole1_3 = [0.0, 0.0]

        self.pole2_1 = [0.0, 0.0]
        self.pole2_2 = [0.0, 0.0]

        self.pole3 = [0.0, 0.0]

        #self.ang_target = 0.0
 
        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'odometry_rotary', self.odometry_callback, 10)

        self.angle_publisher = self.create_publisher(Float32, 'pole_angle', 10)

        self.pole_cmd_subscriber = self.create_subscription(String, 'cmd_goal', self.pole_cmd_cb, 10)


    
    def odometry_callback(self, odom_msg):
        self.current_x = odom_msg.data[0]
        self.current_y = odom_msg.data[1]
        #print(self.current_x, self.current_y)
    def pole_cmd_cb(self, pole_msg):
        ang_msg = Float32()
        if pole_msg == "pole1_1":
            ang_msg.data = np.arctan2(self.current_y-self.pole1_1[1], self.current_x-self.pole1_1[0])
            print(np.arctan2(self.current_y-self.pole1_1[1], self.current_x-self.pole1_1[0]))
        if pole_msg == "pole1_2":
            ang_msg.data = np.arctan2(self.current_y-self.pole1_2[1], self.current_x-self.pole1_2[0])
        self.angle_publisher.publish(ang_msg)


def main(args=None):
    rclpy.init(args=args)

    node = PoleTargeting()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
