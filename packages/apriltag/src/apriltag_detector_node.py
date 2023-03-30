#!/usr/bin/env python3
import math

import cv2
import rospy
import numpy as np
import apriltag
from threading import Thread
from concurrent.futures import ThreadPoolExecutor
import os

import std_msgs.msg
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

MIN_AREA_TO_DEtECT = 2000

CONSTRUCTION_SITE_ID = 12
TRAFFIC_LIGHT_ID = 69


def findArea(corners):
    length1 = math.sqrt((corners[0][0] - corners[1][0]) ** 2 + (corners[0][1] - corners[1][1]) ** 2)
    length2 = math.sqrt((corners[1][0] - corners[2][0]) ** 2 + (corners[1][1] - corners[2][1]) ** 2)
    return length2 * length1


def plain_data():
    return Int32MultiArray(data=[1])


class AprilTagDetector(DTROS):
    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name="apriltag_detector_node", node_type=NodeType.PERCEPTION
        )
        self.bot_name = os.environ["VEHICLE_NAME"]
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
        self.traffic_light_april_tag_pub = rospy.Publisher(
            '~traffic_light_ap_tag', std_msgs.msg.String, queue_size=1
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

    def _find_april_tags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.detector.detect(gray)

    def find_tag(self, target_tag_id, markers, img=None):
        marker_id = [i.tag_id for i in markers]
        if target_tag_id not in marker_id:
            return
        marker_corners = [i.corners for i in markers]

        size_of_detected_area = findArea(corners=marker_corners[marker_id.index(target_tag_id)])
        if size_of_detected_area <= MIN_AREA_TO_DEtECT:
            # do nothing, tag is not close enough
            return
        self.log(f'detected marker from apriltag {target_tag_id}, with area {size_of_detected_area}')
        if target_tag_id == CONSTRUCTION_SITE_ID:
            self.construction_april_tag_pub.publish(plain_data())
        elif target_tag_id == TRAFFIC_LIGHT_ID:
            message = std_msgs.msg.String()
            msg = find_traffic_light_color(cropped_image=crop_traffic_light_img(img, marker_corners[0]))
            print(msg)
            message.data = msg
            if msg == "red":
                os.system(f'rosparam set /{self.bot_name}/kinematics_node/gain 0.0')
            else:
                os.system(f'rosparam set /{self.bot_name}/kinematics_node/gain 1.0')
            self.traffic_light_april_tag_pub.publish(message)

    def cb_image(self, msg):
        if self.start_detect:
            img = self.bridge.compressed_imgmsg_to_cv2(msg)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            markers = self._find_april_tags(img)
            marker_id = [i.tag_id for i in markers]
            marker_corners = [i.corners for i in markers]
            self.log(f'detected marker from apriltag {marker_id}')
            for i in markers:
                self.log(f'detected marker {i.corners}')
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
                markers = self._find_april_tags(img)
                self.find_tag(CONSTRUCTION_SITE_ID, markers)
                self.find_tag(TRAFFIC_LIGHT_ID, markers, img)
                self.counter = 1
            else:
                self.counter += 1


def find_mean_hue(image):
    counter = 0
    _sum = 0
    for sample in image:
        for element in sample:
            if element[2] == 0 & element[1] == 0:
                continue
            counter += 1
            if element[0] > 160:
                _sum += 180 - element[0]
            else:
                _sum += element[0]
    return _sum // counter


def find_traffic_light_color(cropped_image):
    lower_bound = np.array([0, 0, 175], dtype="uint8")
    higher_bound = np.array([255, 200, 255], dtype="uint8")
    mask = cv2.inRange(cv2.cvtColor(cropped_image, cv2.COLOR_RGB2HSV), lower_bound, higher_bound)
    detected_colors = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)
    if find_mean_hue(cv2.cvtColor(detected_colors, cv2.COLOR_RGB2HSV)) < 30:
        return "red"
    else:
        return "green"


def calculatePointAbove(bottom, top):
    # Get the slope and intercept of the line L
    x1, y1 = top
    x2, y2 = bottom
    m = 0 if (x2 - x1 == 0) else (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    dy = y1 - y2
    y3 = (y1 + dy) * 1.0
    x3 = (y3 - b) / m
    new_top = (int(x3), int(y3))
    return new_top


def calculate_corners_of_traffic_lights(img, atag_detection_corners):
    corners = atag_detection_corners
    #
    #
    # new_tl                 new_tr
    #
    #   ~traffic light is here~
    #
    # [0]                     [1]
    #
    #     ~april tag is here~
    #
    #
    # [3]                      [2]
    #
    new_tl = calculatePointAbove(corners[3], corners[0])
    new_tr = calculatePointAbove(corners[2], corners[1])
    new_corners = new_tl, new_tr, tuple(corners[1]), tuple(corners[0])
    return new_corners


def crop_traffic_light_img(img, atag_detection_corners):
    """
            Returns the image cropped to the square of the traffic lights

            Parameters:
                img: input image (as a NumPy array).
                atag_detection_corners: the detection corners of the april tag below the traffic light, as returned by the detector.detect() method

            Returns:
                The cropped image (as a NumPy array).
            """
    (tl, tr, br, bl) = calculate_corners_of_traffic_lights(img, atag_detection_corners)
    # Define three non-collinear points in the source image (in clockwise order, starting from tl)
    src_pts = np.array([tl, tr, bl], dtype=np.float32)

    # Define the corresponding points in the destination image (top-left, top-right, bottom-left respectively)
    dst_width = int(np.round(np.sqrt((bl[1] - tl[1]) ** 2 + (bl[0] - tl[0]) ** 2)))
    dst_height = int(np.round(np.sqrt((tr[1] - tl[1]) ** 2 + (tr[0] - tl[0]) ** 2)))
    dst_pts = np.array([(0, 0), (dst_width, 0), (0, dst_height)], dtype=np.float32)

    # Compute the affine transform matrix
    M = cv2.getAffineTransform(src_pts, dst_pts)

    # Apply the transform to the source image, cropping it
    cropped_img = cv2.warpAffine(img, M, (dst_width, dst_height))

    # Return the cropped image
    return cropped_img


if __name__ == "__main__":
    node = AprilTagDetector()
    # spin forever
    rospy.spin()
