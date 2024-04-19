#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import time
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import cv2
from std_msgs.msg import Int32MultiArray
import os
import torch
from pysot.core.config import cfg
from pysot.models.model_builder import ModelBuilder
from pysot.tracker.tracker_builder import build_tracker
from pysot.utils.model_load import load_pretrain
from std_msgs.msg import Bool

#pysot config
current_path = os.path.dirname(os.path.realpath(__file__))
CONFIG_PATH = os.path.join(current_path, 'pysot/siamrpn_r50_l234_dwxcorr_lt/config.yaml')
MODEL_PATH = os.path.join(current_path, 'pysot/siamrpn_r50_l234_dwxcorr_lt/model.pth')

class MessageItem(object):
    def __init__(self, frame, message, lost):
        self._frame = frame
        self._message = message
        self._lost = lost

    def getFrame(self):
        return self._frame

    def getMessage(self):
        return self._message
    
    def getStatus(self):
        return self._lost

    def getROI(self):
        if self._message and 'coord' in self._message:
            (x1, y1), (x2, y2) = self._message['coord'][0]
            w = x2 - x1
            h = y2 - y1
            return (x1, y1, w, h)
        return None

class Tracker(object):
    def __init__(self,draw_coord=True):
        self.draw_coord = draw_coord
        self.lost = False
        cfg.merge_from_file(CONFIG_PATH)
        cfg.CUDA = torch.cuda.is_available() and cfg.CUDA
        device = torch.device('cuda' if cfg.CUDA else 'cpu')

        # create model
        model = ModelBuilder()

        # load model
        model.load_state_dict(torch.load(MODEL_PATH, map_location=lambda storage, loc: storage.cpu()))
        model.eval().to(device)

        # build tracker
        self.tracker = build_tracker(model)

    def initWorking(self, frame, box):
        status = self.tracker.init(frame, box)
        self.coord = box
        self.isWorking = True

    
    def track(self, frame):
        message = None
        if self.isWorking:
            outputs = self.tracker.track(frame)
            # print(outputs)
                                        
            if outputs['best_score'] > cfg.TRACK.CONFIDENCE_LOW:
                self.lost = False
                if 'polygon' in outputs:
                    polygon = np.array(outputs['polygon']).astype(np.int32)
                    cv2.polylines(frame, [polygon.reshape((-1, 1, 2))],
                                True, (0, 255, 0), 3)
                    mask = ((outputs['mask'] > cfg.TRACK.MASK_THERSHOLD) * 255)
                    mask = mask.astype(np.uint8)
                    mask = np.stack([mask, mask*255, mask]).transpose(1, 2, 0)
                    frame = cv2.addWeighted(frame, 0.77, mask, 0.23, -1)
                else:
                    bbox = list(map(int, outputs['bbox']))
                    self.coord = (bbox[0], bbox[1], bbox[2], bbox[3])
                    message = {"coord": [((bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]))]}
                    if self.draw_coord:
                        p1 = (int(bbox[0]), int(bbox[1]))
                        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                        cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                        message['msg'] = "is tracking"
            else:
                self.lost = True
                cv2.putText(frame, "Searching targets", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        return MessageItem(frame, message, self.lost)



class TrackerNode:
    def __init__(self):
        rospy.init_node('tracker_siam_node', anonymous=True)
        self.bridge = CvBridge()

        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/tb3_0/camera/rgb/image_raw', Image, self.image_callback, queue_size=10, buff_size=2**24)
        # 添加深度图像的订阅者
        self.depth_sub = rospy.Subscriber('/tb3_0/camera/depth/image_raw', Image, self.depth_callback, queue_size=10, buff_size=2**24)
        self.depth_image = None
        
        # 发布跟踪结果的话题
        self.distance_pub = rospy.Publisher('/tracker/distance', Float32, queue_size=10)
        self.coord_pub = rospy.Publisher('/tracker/coordinate', Point, queue_size=10)
        self.status_pub = rospy.Publisher('/tracker/status', Bool, queue_size=10)
        self.offset_pub = rospy.Publisher('/tracker/offset', Float32, queue_size=10)

        
        self.gTracker = None
        self.is_initialized = False
        self.last_known_roi = None
        self.last_known_distance = None

    def depth_callback(self, data):
        try:
            # 将深度图像转换为 NumPy 数组
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            # rospy.loginfo("Depth image shape: {0}".format(self.depth_image.shape))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def image_callback(self, data):
        if not self.is_initialized:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                # rospy.loginfo("Color image shape: {0}".format(cv_image.shape))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
                return
            
            
            cv2.imshow("Image window", cv_image)
            key = cv2.waitKey(1)
            
            if key == ord(' '):  # 检测到空格键被按下
            # 在这里调用cv2.selectROI来选择ROI
                rospy.loginfo("Select ROI on the image window and press ENTER or SPACE when done. Press 'c' to cancel.")
                gROI = cv2.selectROI(cv_image, False)
                if any(gROI):  # 检查ROI是否有效（非零）
                    self.gTracker = Tracker()
                    self.gTracker.initWorking(cv_image, gROI)
                    self.is_initialized = True
                    rospy.loginfo("ROI selected and tracker initialized.")
                    cv2.destroyAllWindows()
                else:
                    rospy.loginfo("ROI selection canceled.")
                    return

            # 关闭选择ROI的窗口

        
        # 如果已经初始化，处理图像
        if self.is_initialized:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
                return
            
            # 这里实现追踪逻辑
            _item = self.gTracker.track(cv_image)
            cv2.imshow("track result", _item.getFrame())          
            key = cv2.waitKey(1)
            if key == 27:  # 如果用户按下了ESC键27
                self.is_initialized = False
                cv2.destroyAllWindows()

            #tracking statue
            lost = _item.getStatus()
            self.status_pub.publish(lost)


            
            #if the object is not lost, publish the coordinate
            if lost == False:
                #tracking coordinate
                box = _item.getROI()
                x, y, w, h = box
                # # 使用 ROI 的坐标来获取深度值
                # roi_depth = self.depth_image[y:y+h, x:x+w]
                # 计算 ROI 中心的坐标
                center_x, center_y = x + w // 2, y + h // 2

                # 计算新的 ROI 的宽度和高度，取原来的一半
                new_w, new_h = w // 2, h // 2

                # 计算新的 ROI 的左上角坐标
                new_x, new_y = center_x - new_w // 2, center_y - new_h // 2

                # 使用新的 ROI 的坐标来获取深度值
                roi_depth = self.depth_image[new_y:new_y+new_h, new_x:new_x+new_w]

                # 计算新的 ROI 的平均深度
                mean_depth = np.nanmean(roi_depth)

                # 计算 ROI 的平均深度
                mean_depth = np.nanmean(roi_depth)
                # 创建一个新的 Float32 消息
                distance_msg = Float32()
                distance_msg.data = mean_depth
                # 发布深度值
                self.distance_pub.publish(distance_msg)
                
                #calculate and publish the offset
                image_center_x = cv_image.shape[1]  / 2.0
                box_center_x = x + w / 2.0
                offset_x_pixels = float(box_center_x - image_center_x)

                # 从相机参数中获取焦距
                focal_length_pixels = 1206.0

                # 使用相机的焦距和物体的实际深度，将像素偏移量转换为实际的物理偏移量
                offset_x_meters = (offset_x_pixels * mean_depth) / focal_length_pixels
                
                offset_msg = Float32()
                offset_msg.data = offset_x_meters
                self.offset_pub.publish(offset_msg)
                
                # 计算偏移角度
                offset_angle = np.arctan(offset_x_meters / mean_depth)
                offset_angle_degrees = np.degrees(offset_angle)
                # rospy.loginfo("Offset angle: {0}".format(offset_angle_degrees))



                



if __name__ == '__main__':
    try:
        tracker_node = TrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
