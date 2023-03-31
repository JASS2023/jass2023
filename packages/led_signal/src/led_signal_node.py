#!/usr/bin/env python3
import os
import time
import rospy
import std_msgs
import jsonify

from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA, Int32MultiArray, Float32MultiArray, String

from duckietown.dtros import DTROS, TopicType, NodeType

YELLOW = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [
    0.0, 0.0, 0.0], [0.4, 0.3, 1], [1, 1, 0.1]]
ORANGE = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0], [1, 0.6, 0], [1, 1, 1]]


BASIC = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [
    0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]


class LEDSignalNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDSignalNode, self).__init__(
            node_name=node_name, node_type=NodeType.DRIVER)

        self.LEDsPattern = [[0.0, 0.0, 1.0]] * 5

        # Publisher
        self.pub_leds = rospy.Publisher(
            "~led_pattern", LEDPattern, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # Subscriber
        self.april_tags_sub = rospy.Subscriber("~construction_ap_tag", Int32MultiArray,
                                               self.on_detect_construction_sign, queue_size=1)

        self.traffic_light_atags_sub = rospy.Subscriber("~traffic_light_ap_tag", std_msgs.msg.String,
                                                        self.on_detect_traffic_light, queue_size=1)

        self.context_sub = rospy.Subscriber(
            "~color", String, self.change_led, queue_size=1)

        self.is_in_zone = 1

    """
    {   
        "type": "zone",
        "data": {
            "value": STRING
        }
    }
    """

    def change_led(self, msg):
        if msg.data["value"] == "in_zone":
            self.change_color(YELLOW, sleep_time=0.15)
            self.change_color(BASIC, sleep_time=0.15)
        else:
            self.change_color(BASIC)


    def on_detect_traffic_light(self, msg):
        if msg.data == "red":
            self.change_color([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [
                              0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        else:
            self.change_color([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [
                              0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])

    def on_detect_construction_sign(self, msg):
        self.change_color([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [
                          0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        self.change_color([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [
                          0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])

    def change_color(self, led_pattern, sleep_time=1):
        '''

        Args:
            led_pattern: - pattern for LEDs, list of 5 lists with RGB, 4th and 5th items are for the back LEDs

        Returns:

        '''
        self.LEDsPattern = led_pattern
        self.publish_led(sleep_time)

    def publish_led(self, sleep_time=1):
        """
        Publishes new colors for LEDs into /{bot_name}/led_emitter/led_pattern topic
        """
        LEDPattern_msg = LEDPattern()
        for k in range(1):
            for i in range(5):
                rgba = ColorRGBA()
                rgba.r = self.LEDsPattern[i][0]
                rgba.g = self.LEDsPattern[i][1]
                rgba.b = self.LEDsPattern[i][2]
                rgba.a = 1.0
                LEDPattern_msg.rgb_vals.append(rgba)
                self.pub_leds.publish(LEDPattern_msg)
            rospy.sleep(sleep_time)


if __name__ == "__main__":
    led_signal_node = LEDSignalNode(node_name="led_emitter")
    rospy.spin()
