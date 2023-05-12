import cv2
import numpy as np
import rclpy
import pyrealsense2 as rs


from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge




class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Configuration of realsense
        #self.pipeline = rs.pipeline()
        #self.config = rs.config()
        # Get device product line
        #self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        #self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        #self.device = self.pipeline_profile.get_device()
        #self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        #self.found_rgb = False
        #self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #self.pipeline.start(self.config)
        # Configuration of opencv
        self.capture = cv2.VideoCapture(4)
        self.cv_bridge = CvBridge()
        # Publisher from Realsense
        self.image_publisher_rgb = self.create_publisher(Image, 'camera/input/rgb', 100)
        self.image_publisher_depth = self.create_publisher(Image, 'camera/input/depth', 100)
        self.image_timer = self.create_timer(0.05, self.image_timer_callback)
 

    def image_timer_callback(self):
        
        ret, frame = self.capture.read()


        print(frame.shape)

        frame = np.asarray(frame)
        img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.image_publisher_rgb.publish(img_msg)
        self.get_logger().info("Publishing data from camera ...")


def main(args=None):
    rclpy.init(args=args)

    node_image = ImagePublisher()

    rclpy.spin(node_image)

    node_image.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
     
