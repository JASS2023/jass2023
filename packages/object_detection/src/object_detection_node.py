#!/usr/bin/env python3
from __future__ import annotations

import rospy
import pickle
from time import perf_counter
import requests

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage, Float32MultiArray
import json

COUNTER_FREQUENCY = 10

IP = '192.168.0.176'
PORT = 8000


class ObjectDetectionNode(DTROS):
    def __init__(self) -> None:
        self.counter = 0
        super(ObjectDetectionNode, self).__init__(
            node_name="object_detection_node", node_type=NodeType.PERCEPTION
        )

        self._img_sub = rospy.Subscriber(
            "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        )

        self._detected_objs = rospy.Publisher(
            "~objects_detected", Float32MultiArray, queue_size=1, dt_topic_type=TopicType.DRIVER
        )
        

    def cb_image(self, msg) -> None:        
        self.counter += 1
        self.log(f"Received {self.counter} images")
        if self.counter == COUNTER_FREQUENCY:
            self.counter = 0
            
            # Capture start time
            t_start = perf_counter()
            
            send_package = pickle.dumps(msg.data)
            self.log(f"Sending package of size {len(send_package)}")
        
            # Send the image to the server and receive the response
            response = requests.post(f"http://{IP}:{PORT}/detect_objects", data=send_package)
            response = json.loads(response)

            if response is None:
                return

            # Capture end time
            t_end = perf_counter()

            # Not sure what kind of objects will be sent (if it is an array of integers, it will be awful)
            self.log(f"Received response: {response} after {t_end - t_start} seconds")
            
            # mb receive "the most dangerous" object
            reply = Float32MultiArray()
            reply.data = response
            self._detected_objs.publish(reply)


if __name__ == "__main__":
    node = ObjectDetectionNode()
    # spin forever
    rospy.spin()
