#!/usr/bin/env python3
import os
import time
import rospy

from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.srv import SetCustomLEDPatternResponse, ChangePatternResponse
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int32MultiArray

from duckietown.dtros import DTROS, TopicType, NodeType


class LEDSignalNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDSignalNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)
        self.LEDspattern = [[0.0, 0.0, 1.0]] * 5
        self.pub_leds = rospy.Publisher(
            "~led_pattern", LEDPattern, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # self._img_sub = rospy.Subscriber(
        #     "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        # )
        self.april_tags_sub = rospy.Subscriber("~construction_ap_tag", Int32MultiArray, self.onDetectAprilTag, queue_size=1)
        self.traffic_light_atags_sub = rospy.Subscriber("~traffic_light_ap_tag", Int32MultiArray, self.onDetectedTrafficLight, queue_size=1)
        # traffic_light_ap_tag
        # self.publishLEDs()


    def onDetectedTrafficLight(self, message):
        self.changeColor([[1.0, 0.0, 1.0]] * 5)
        self.changeColor([[0.0, 1.0, 0.0]] * 5)
    def onDetectAprilTag(self, message):
        self.changeColor([[0.0, 1.0, 0.0]] * 5)
        self.changeColor([[0.0, 0.0, 1.0]] * 5)
        # self.LEDspattern = [[0.0, 1.0, 0.0]] * 5
        # self.publishLEDs()
        # self.LEDspattern = [[0.0, 0.0, 1.0]] * 5
        # self.publishLEDs()
    def changeColor(self, led_pattern):
        self.LEDspattern = led_pattern
        self.publishLEDs()

    def publishLEDs(self):
        LEDPattern_msg = LEDPattern()
        for k in range(1):
            for i in range(5):
                rgba = ColorRGBA()
                rgba.r = self.LEDspattern[i][0]
                rgba.g = self.LEDspattern[i][1]
                rgba.b = self.LEDspattern[i][2]
                rgba.a = 1.0
                LEDPattern_msg.rgb_vals.append(rgba)
                self.pub_leds.publish(LEDPattern_msg)
            rospy.sleep(1)


if __name__ == "__main__":
    led_signal_node = LEDSignalNode(node_name="led_emitter")
    rospy.spin()
