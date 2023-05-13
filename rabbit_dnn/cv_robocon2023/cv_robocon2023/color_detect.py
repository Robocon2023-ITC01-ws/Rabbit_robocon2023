import rclpy
import numpy as np
import cv2
import pyrealsense2 as rs

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from cv_robocon2023.pid_controller import PIDController


class ColorDetect(Node):

    def __init__(self):
        super().__init__('color_node')

        self.image_subscriber = self.create_subscription(Image, 'camera/input/rgb', self.image_callback, 10)
        self.color_publisher = self.create_publisher(Image, 'camera/rgb/color', 10)
        self.color_timer = self.create_timer(0.066, self.color_callback)
        self.cv_bridge = CvBridge()

        ## Object coordinate
        self.x = 0.0
        self.y = 0.0
        self.w = 0.0
        self.h = 0.0

        self.image_color = np.zeros((640, 480))


        self.target_height = np.zeros(240)
        self.target_width = np.zeros(320)

        ### PID values

        ## height
        self.kp_h = 0.0
        self.ki_h = 0.0
        self.kd_h = 0.0
        ## width
        self.kp_w = 0.0
        self.ki_w = 0.0
        self.kd_w = 0.0

        self.dt = 0.1

        self.pid_h = PIDController(
            self.kp_h, self.ki_h, self.kd_h, self.dt
        )

        self.pid_w = PIDController(
            self.kp_w, self.ki_w, self.kd_w, self.dt
        )

        self.error_h = [self.target_height-self.h]
        self.error_w = [self.target_width-self.w]

        self.output_h = None
        self.output_w = None


    def image_callback(self, image_msg):
        
        image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        image = np.asarray(image)

        self.target_height = image.shape[0]/2
        self.target_width = image.shape[1]/2

        hsvFrame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        kernel = np.ones((5, 5), np.uint8)


        yellow_lower = np.array([22, 93, 0])
        yellow_upper = np.array([45, 255, 255])

        yellow_mask = cv2.inRange(
            hsvFrame, yellow_lower, yellow_upper
        )

        yellow_mask = cv2.dilate(yellow_mask, kernel)
        res_yellow = cv2.bitwise_and(
            image, image, mask=yellow_mask
        )

        contours, hierarchy = cv2.findContours(
            yellow_mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 114):
                self.x, self.y, self.w, self.h = cv2.boundingRect(contour)
                image = cv2.rectangle(image, (self.x, self.y),
                                        (self.x + self.w, self.y + self.h),
                                        [45, 255, 255], 2)
                
                cv2.putText(image, f"Yellow Colour {self.x}, {self.y}", (self.x, self.y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, [45, 255, 255])

                self.image_color = image

                # print(imageFrame.shape)
                
    def color_callback(self):

        image_color = np.asarray(self.image_color)
        image_color = self.cv_bridge.cv2_to_imgmsg(image_color)

        self.color_publisher.publish(image_color)
        
    def pid_callback(self):

        self.error_h.append(self.target_height-self.h/2)
        self.error_w.append(self.target_width-self.w/2)

        self.output_h = self.pid_h.calculate_pid(self.error_h)
        self.output_w = self.pid_w.calculate_pid(self.error_w)



def main(args=None):

    rclpy.init(args=args)

    node = ColorDetect()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()