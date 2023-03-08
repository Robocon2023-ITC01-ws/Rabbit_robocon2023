import os
import platform
import sys
from pathlib import Path


import numpy as np
import cv2
import torch
import rclpy


from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from yolov5_msgs.msg import BoundingBox, BoundingBoxes

from cv_robocon2023.models.common import DetectMultiBackend
from cv_robocon2023.utils.dataloaders import VID_FORMATS, LoadStreams, letterbox
from cv_robocon2023.utils.general import (check_img_size, non_max_suppression, scale_boxes, xyxy2xywh)
from cv_robocon2023.utils.plots import Annotator, Colors
from cv_robocon2023.utils.torch_utils import select_device
from cv_robocon2023.utils.torch_utils import select_device, time_sync

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative



class Yolov5:

    def __init__(self,
    	    class_names,
            weights,
            data,
            img_size,
            conf_thres,
            iou_thres,
            max_det,
            device,
            agnostic_nms,
            line_thickness,
            dnn,
            vid_stride=1,
            ):
       
        ## Realsense configuration
        # self.pipeline = rs.pipeline()
        # self.config = rs.config()

        # self.config.enable_stream(rs.stream.depth, 680, 480, rs.foramt.z16, 30)


        self.classes = class_names
        self.weights = weights
        self.data = data
        self.img_size = img_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = device
        self.agnostic_nms = agnostic_nms
        self.line_thickness = line_thickness
        self.dnn = dnn
        self.vid_stride = vid_stride
        self.half = False
        self.s = str()

        #Load model
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=self.dnn, data=self.data, fp16=False)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.img_size = check_img_size(img_size, s=self.stride)
        
        self.dt, self.seen = [0.0, 0.0, 0.0], 0

        # Dataloader
        self.bs = 1 # batch_size
    def image_detect(self, image_raw):
            # frames = pipeline.wait_for_frames()
            # color_frame = frames.get_color_frame()
            # depth = frames.get_depth_frame()

            # if not depth: continue
        
        self.stride = 32  # stride
        self.img_size = 288
    
        colors = Colors()
        img = letterbox(image_raw, self.img_size, stride=self.stride)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(img)

        t1 = time_sync()
        im = torch.from_numpy(im).to(self.model.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        self.dt[0] += t2 - t1

        # Inference

        path = ['0']

        pred = self.model(im, augment=False, visualize=False)
        t3 = time_sync()
        self.dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        self.dt[2] += time_sync() - t3

        # Process predictions
        for i, det in enumerate(pred):
            im0 = image_raw
            self.s += f'{i}: '

            # p = Path(str(p))  # to Path
            self.s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            # imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    self.s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    save_conf = False
                    line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                    x1, x2 = int((int(xyxy[0])+int(xyxy[2]))/2), int((int(xyxy[1])+int(xyxy[3]))/2)
                    # distance = depth.get_distance(int(x1),int(x2))

                    # Add bbox to image
                    c = int(cls)  # integer class
                    label = f'{self.names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))

                    # print(xyxy, label)

            # Stream results
            im0 = annotator.result()

        return im0
     
    



class Yolov5ROS(Node):

    def __init__(self):
        super().__init__('yolov5_node')
        
        ### Declare parameters for Yolov5
        self.declare_parameter('class_names', None)
        self.declare_parameter('weights', '/home/kenotic/ros2dnn_ws/src/cv_robocon2023/cv_robocon2023/config/best_v2.pt')
        #self.declare_parameter('weights', str(ROOT) + 'config/best_v2.pt')
        self.declare_parameter('data',  str(ROOT) + 'data/coco128.yaml')
        self.declare_parameter('img_size', (288, 288))
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('max_det', 100)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('agnostic_nms', False)
        self.declare_parameter('line_thickness', 3)
        self.declare_parameter('dnn', False)
        self.declare_parameter('vid_stride', 1)

        ### Get parameters value
        self.class_names = self.get_parameter('class_names').value
        self.weights = self.get_parameter('weights').value
        self.data = self.get_parameter('data').value
        self.img_size = self.get_parameter('img_size').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.iou_thres = self.get_parameter('iou_thres').value
        self.max_det = self.get_parameter('max_det').value
        self.device = self.get_parameter('device').value
        self.agnostic_nms = self.get_parameter('agnostic_nms').value
        self.line_thickness = self.get_parameter('line_thickness').value
        self.dnn = self.get_parameter('dnn').value
        self.vid_stride = self.get_parameter('vid_stride').value

        
        ### Load Yolov5 Model

        self.yolov5_node = Yolov5(
                self.class_names,
                self.weights,
                self.data,
                self.img_size,
                self.conf_thres,
                self.iou_thres,
                self.max_det,
                self.device,
                self.agnostic_nms,
                self.line_thickness,
                self.dnn,
                self.vid_stride)

        ### Declaring ros master
        

        self.image_subscriber = self.create_subscription(Image, 'camera/input/rgb', self.image_callback, 100)
        self.detected_publisher = self.create_publisher(Image, 'yolov5_output', 100)
        self.label_publisher = self.create_publisher(String, 'yolov5_label', 100)
        ### CV2 Converter
        self.cv_bridge = CvBridge()

        ## Realsense config
    def image_callback(self, image):
        image = self.cv_bridge.imgmsg_to_cv2(image)
    
        image = np.asarray(image)

        result = self.yolov5_node.image_detect(image)

        image_msg = self.cv_bridge.cv2_to_imgmsg(result, encoding="bgr8")
        

        self.detected_publisher.publish(image_msg)
        self.get_logger().info("Publishing result.....")



def main(args=None):
    rclpy.init(args=args)

    yolov5_node = Yolov5ROS()

    rclpy.spin(yolov5_node)

    yolov5_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
