import numpy as np
import cv2
import pyrealsense2 as rs
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class RealsenseConfig:

    def __init__(self):

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start stream
        self.pipeline.start(self.config)    
        
        self.wait_for_frames()

    def wait_for_frames(self):
        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))
        finally:
            self.pipeline.stop()
        return color_image, depth_colormap

                
        
class ImagePublisher(Node):
    def __init__(self, image):
        super().__init__('camera_node')

        ## Create ROS2 Master
        

        self.image_pubRGB = self.create_publisher(Image, 'camera/input/rgb', 100)
        self.image_pubDEPTH = self.create_publisher(CompressedImage, 'camera/input/depth', 100)
        self.image_timer = self.create_timer(0.01, self.image_callbackRGB)
        
        ## Cv bridge
        self.cv_bridge = CvBridge()

        self.image = image
    def image_callbackRGB(self):
        img_msg = self.cv_bridge.cv2_to_imgmsg(self.image, encoding="bgr8")

        self.image_pubRGB.publish(img_msg)

        self.get_logger().info("Publishing image data .....")




def main(args=None):
    rclpy.init(args=args)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            image = np.array(color_image)
            camera_node = ImagePublisher(image)


            rclpy.spin(camera_node)
            camera_node.destroy_node()
        
        rclpy.shutdown()
    finally:

    # Stop streaming
        pipeline.stop()

if __name__=="__main__":
    main()