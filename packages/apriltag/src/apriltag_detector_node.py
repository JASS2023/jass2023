#!/usr/bin/env python3
import math

import cv2
import rospy
import numpy as np
import apriltag
from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Transform, Vector3, Quaternion
from duckietown_msgs.msg import BoolStamped

MIN_AREA_TO_DEtECT = 3000


def findArea(corners):
    length1 = math.sqrt((corners[0][0] - corners[1][0]) ** 2 + (corners[0][1] - corners[1][1]) ** 2)
    length2 = math.sqrt((corners[1][0] - corners[2][0]) ** 2 + (corners[1][1] - corners[2][1]) ** 2)
    return length2 * length1


class AprilTagDetector(DTROS):
    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name="apriltag_detector_node", node_type=NodeType.PERCEPTION
        )
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.start_detect = False
        # self.start_regular_detect = False
        self.bridge = CvBridge()
        self._img_sub = rospy.Subscriber(
            "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        )
        self.marker_id_pub = rospy.Publisher(
            '~tags_id', Int32MultiArray, queue_size=1
        )

        self.construction_april_tag_pub = rospy.Publisher(
            '~construction_ap_tag', Int32MultiArray, queue_size=1
        )
        self.counter = 0
        self.stop_sub = rospy.Subscriber(
            '~start_detection', BoolStamped, self.change_stop_val, queue_size=1
        )

        self.start_sub = rospy.Subscriber(
            '~stop_detection', Bool, self.change_start_val, queue_size=1
        )

        self.switcher_sub = rospy.Subscriber(
            '~switcher', Bool, self.update_switcher, queue_size=1
        )
        self.log('apriltag_init')
        self.switcher = True

    def update_switcher(self, msg):
        self.switcher = True

    # def change_regular_detect(self):
    #     self.start_regular_detect = not self.regular_detect

    def change_start_val(self, msg):
        self.log("stop detection")
        self.start_detect = False

    def change_stop_val(self, msg):
        if self.switcher:
            self.start_detect = True
            self.switcher = False

    def _findAprilTags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def cb_image(self, msg):
        if self.start_detect:
            img = self.bridge.compressed_imgmsg_to_cv2(msg)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            markers = self._findAprilTags(img)
            marker_id = [i.tag_id for i in markers]
            marker_corners = [i.corners for i in markers]
            self.log(f'detected marker from apriltag {marker_id}')
            for i in markers:
                self.log(f'detected marker {i.corners}')
            # self.log(f'detected marker {markers}')
            # self.log(marker_id)
            if len(marker_id) != 0:
                if 12 in marker_id:
                    self.log(f'detected marker from apriltag {12}')
                    if findArea(corners=marker_corners[marker_id.index(12)]) > MIN_AREA_TO_DEtECT:
                        self.log(f'successful')
                        self.construction_april_tag_pub.publish(Int32MultiArray(data=[1]))
                else:
                    marker_msg = Int32MultiArray(data=marker_id)
                    self.marker_id_pub.publish(marker_msg)
                    self.start_detect = False
        else:
            if self.counter % 6 == 0:
                img = self.bridge.compressed_imgmsg_to_cv2(msg)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                markers = self._findAprilTags(img)
                marker_id = [i.tag_id for i in markers]
                marker_corners = [i.corners for i in markers]
                # self.log(f'detected marker from apriltag {marker_id}')
                if len(marker_id) != 0:
                    if 12 in marker_id:
                        self.log(f'detected marker from apriltag {12}')
                        if findArea(corners=marker_corners[marker_id.index(12)]) > MIN_AREA_TO_DEtECT:
                            self.log(f'successful, area is {findArea(corners=marker_corners[marker_id.index(12)])}')
                            self.construction_april_tag_pub.publish(Int32MultiArray(data=[1]))
                self.counter = 1
            else:
                self.counter += 1


if __name__ == "__main__":
    node = AprilTagDetector()
    # spin forever
    rospy.spin()


