import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from ocp_robocon2023.tf_transformations import euler_from_quaternion, quaternion_from_euler
from ocp_robocon2023.elephant_robot import ElephantModel


class TestPublisher(Node):

	def __init__(self):
		super().__init__('test_pub')
		self.publisher_ = self.create_publisher(String, 'topic', 10)
		self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
		self.subscriber_control = self.create_subscription(
			Float32MultiArray,
			'pub_speed',
			self.control_callback,
			10
		)
		self.elephant_model = ElephantModel()
		timer_period = 0.1
		self.timer_ = self.create_timer(timer_period, self.odom_callback)
		self.i = 0

		self.current_x = 0.0
		self.current_y = 0.0
		self.current_th = 0.0

	def control_callback(self, msg):
		v1 = msg.data[0]
		v2 = msg.data[1]
		v3 = msg.data[2]
		v4 = msg.data[3]
		vx, vy, vth = self.elephant_model.forward_kinematic(v1, v2, v3, v4, 0.0, "numpy")
		self.current_x += vx * 0.1
		self.current_y += vy * 0.1
		self.current_th += vth * 0.1
		# self.get_logger().info('Publishing: "%s"' % msg.data)
		# print(self.current_x, self.current_y, self.current_th)


	def timer_callback(self):
		msg = String()
		msg.data = 'Hello world!: %d' % self.i
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg.data)
		self.i += 1

	def odom_callback(self):
		odom = Odometry()
		x = self.current_x
		y = self.current_y
		th = self.current_th
		q0, q1, q2, q3 = quaternion_from_euler(0.0, 0.0, th, 'rxyz')
		odom.pose.pose.position.x = x
		odom.pose.pose.position.y = y
		odom.pose.pose.orientation.x = q0
		odom.pose.pose.orientation.y = q1
		odom.pose.pose.orientation.z = q2
		odom.pose.pose.orientation.w = q3
		print(x, y, th)
		self.publisher_odom.publish(odom)

def main(args=None):
	rclpy.init(args=args)
	test_publisher = TestPublisher()
	rclpy.spin(test_publisher)

	test_publisher.destroy_node()
	rclpy.shutdown()



if __name__=="__main__":
	main()
