import numpy as np
import cv2
import rclpy
import pyrealsense2 as rs
from PIL import Image as img

# from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class Realsense(Node):

    def __init__(self):

        super().__init__('realsense_node')

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start stream
        self.pipeline.start(self.config)

        self.image = np.zeros((640, 480))
        self.color = np.zeros((640, 480))

        self.color_image = np.zeros((640, 480))

        self.cv_bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, 'camera/input/rgb', 10)
        self.color_pub = self.create_publisher(Image, 'camera/output/color', 10)

        self.realsense_timer = self.create_timer(0.04, self.wait_for_frames)
        self.color_timer = self.create_timer(0.05, self.color_callback)



        self.opt_distance = []
        self.opt_x = []
        self.opt_y = []

        self.opt = np.concatenate([self.opt_x, self.opt_y, self.opt_distance])
        

    def wait_for_frames(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # if not depth_frame or not color_frame:
        #     continue

        # Convert images to numpy arrays

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Publish
        img_msg = self.cv_bridge.cv2_to_imgmsg(color_image, encoding="bgr8")

        self.image_pub.publish(img_msg)

        # image = np.asarray(color_image)

        self.color = np.asarray(color_image)

        hsvFrame = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)

        kernel = np.ones((5, 5), "uint8")

        yellow_lower = np.array([22, 93, 0])
        yellow_upper = np.array([45, 255, 255])

        yellow_mask = cv2.inRange(
            hsvFrame, yellow_lower, yellow_upper
        )

    
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        yellow_mask = cv2.dilate(yellow_mask, kernel)
        res_yellow = cv2.bitwise_and(
            self.color, self.color, mask=yellow_mask
        )

        contours, hierarchy = cv2.findContours(
            yellow_mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 114):
                x, y, w, h = cv2.boundingRect(contour)
                self.color = cv2.rectangle(self.color, (x, y),
                                        (x + w, y + h),
                                        [45, 255, 255], 2)
                
                dist = depth_frame.get_distance(int(x), int(y))

                self.opt_x.append(x)
                self.opt_y.append(y)
                self.opt_distance.append(dist)

                self.opt = np.vstack([self.opt_x, self.opt_y, self.opt_distance])
                
                dist = np.min(dist)
                x = int(np.min(x))
                y = int(np.min(y))
                
                cv2.putText(self.color, f"distance {dist} m", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, [45, 255, 255])
                cv2.line(self.color, (x, y), (320, 240), [45, 255, 255], 2)

        cv2.drawMarker(self.color, (320, 240), [45, 255, 255], cv2.MARKER_CROSS, 15, 2)



    def color_callback(self):
        
        self.color = np.asarray(self.color)
        color_img = self.cv_bridge.cv2_to_imgmsg(self.color, encoding="bgr8")

        self.color_pub.publish(color_img)


def main(args=None):
    rclpy.init(args=args)

    node_image = Realsense()

    rclpy.spin(node_image)

    node_image.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
