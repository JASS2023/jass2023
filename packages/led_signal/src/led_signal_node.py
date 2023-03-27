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
            "/db01/led_emitter_node/led_pattern", LEDPattern, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # self._img_sub = rospy.Subscriber(
        #     "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        # )
        self.april_tags_sub = rospy.Subscriber("/db01/apriltag_detector_node/tags_id", Int32MultiArray, self.onDetectAprilTag, queue_size=1)
        self.publishLEDs()


    def onDetectAprilTag(self, message):
        self.LEDspattern = [[1.0, 0.0, 0.0]] * 5
        rospy.sleep(5)
        self.LEDspattern = [[0.0, 0.0, 1.0]] * 5

    def publishLEDs(self):
        LEDPattern_msg = LEDPattern()
        while True:
            for i in range(5):
                rgba = ColorRGBA()
                rgba.r = self.LEDspattern[i][0]
                rgba.g = self.LEDspattern[i][1]
                rgba.b = self.LEDspattern[i][2]
                rgba.a = 1.0
                LEDPattern_msg.rgb_vals.append(rgba)
                self.pub_leds.publish(LEDPattern_msg)


if __name__ == "__main__":
    led_signal_node = LEDSignalNode(node_name="led_emitter")
    rospy.spin()
