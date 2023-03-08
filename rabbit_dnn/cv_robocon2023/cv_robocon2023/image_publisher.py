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
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Get device product line
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        self.found_rgb = False
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        # Configuration of opencv
        #self.capture = cv2.VideoCapture(4)
        self.cv_bridge = CvBridge()
        # Publisher from Realsense
        self.image_publisher_rgb = self.create_publisher(Image, 'camera/input/rgb', 100)
        self.image_publisher_depth = self.create_publisher(Image, 'camera/input/depth', 100)
        self.image_timer = self.create_timer(0.01, self.image_timer_callback)
    def image_timer_callback(self):
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))
        
                image_rgb = self.cv_bridge.cv2_to_imgmsg(color_image)
                image_depth = self.cv_bridge.cv2_to_imgmsg(depth_image)

                self.image_publisher_rgb.publish(image_rgb)
                self.image_publisher_depth.publish(image_depth)

                self.get_logger().info("Publishing Data from Realsense.....")
        finally:
            self.pipeline.stop()



def main(args=None):
    rclpy.init(args=args)

    node_image = ImagePublisher()

    rclpy.spin(node_image)

    node_image.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
    
