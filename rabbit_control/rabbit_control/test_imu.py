import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class TestImu(Node):

    def __init__(self):

        super().__init__("test_imu_node")
        
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)

    def map(self, input,  min_input, max_input, min_output, max_output):

        return ((input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)



    def imu_callback(self, imu_msg):

        q1 = imu_msg.orientation.x
        q2 = imu_msg.orientation.y
        q3 = imu_msg.orientation.z
        q4 = imu_msg.orientation.w

        # orient_list = [q1, q2, q3, q4]

        # roll, pitch, yaw = euler_from_quaternion(orient_list)

        # yaw = self.map(yaw, -3.14, 3.14, 0, 6.28)
        siny_cosp = 2 * (q4*q3 + q1*q2)
        cosy_cosp = 1 - 2 * (q2*q2+q3*q3*q3)

        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # if yaw > np.pi:
        #      yaw = yaw + 2 * np.pi

        if yaw < -np.pi:
            yaw = yaw + 2 * np.pi

        print(yaw)


def main(args=None):

    rclpy.init(args=args)

    node = TestImu()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()