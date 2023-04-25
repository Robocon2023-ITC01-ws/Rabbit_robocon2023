import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from ocp_robocon2023.tf_transformations import euler_from_quaternion, quaternion_from_euler
from ocp_robocon2023.rabbit_robot import RabbitModel


class TestPublisher(Node):

	def __init__(self):
		super().__init__('test_pub')

		self.publisher_odom = self.create_publisher(Vector3, 'odometry', 10)
		self.subscriber_control = self.create_subscription(
			Float32MultiArray,
			'input_controls',
			self.control_callback,
			10
		)

		self.timer_ = self.create_timer(0.1, self.odom_callback)
		self.publish_control = self.create_publisher(Float32MultiArray, 'feedback_controls', 10)
		self.rabbit_model = RabbitModel()

		self.current_x = 0.0
		self.current_y = 0.0
		self.current_th = 0.0

		self.v1 = 0.0
		self.v2 = 0.0
		self.v3 = 0.0
		self.v4 = 0.0

	def control_callback(self, msg):
		self.v1 = msg.data[0]
		self.v2 = msg.data[1]
		self.v3 = msg.data[2]
		self.v4 = msg.data[3]
		vx, vy, vth = self.rabbit_model.forward_kinematic(self.v1, self.v2, self.v3, self.v4, 0.0, "numpy")

		self.current_x = self.current_x + vx * 0.1
		self.current_y += self.current_y + vy * 0.1
		self.current_th += self.current_th + vth * 0.1

		# print(self.current_x ,self.current_y, self.current_th)
		print(vx, vy, vth)

	def control_pub(self):
		input_msg = Float32MultiArray()
		input_msg.data = [self.v1, self.v2, self.v3, self.v4]
		self.publish_control.publish(input_msg)


	def odom_callback(self):
		odom = Vector3()
		odom.x = self.current_x
		odom.y = self.current_y
		odom.z = self.current_th
		self.publisher_odom.publish(odom)

def main(args=None):
	rclpy.init(args=args)
	test_publisher = TestPublisher()
	rclpy.spin(test_publisher)

	test_publisher.destroy_node()
	rclpy.shutdown()



if __name__=="__main__":
	main()
