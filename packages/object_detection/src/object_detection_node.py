#!/usr/bin/env python3
from __future__ import annotations

import os
import json
import rospy
import pickle
from time import perf_counter, time
import requests

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


COUNTER_FREQUENCY = 5

IP = '192.168.0.43'
PORT = 8080

CASES = {1: 'on the oncomming trafic',
            2: 'object on the way',
            3: 'object in the roadside',
            4: 'object is too far',
            5: 'emergency stop',
            0: 'nothing is on our way'}


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
            "~objects_detected", String, queue_size=1, dt_topic_type=TopicType.DRIVER
        )
        

    def cb_image(self, msg) -> None:        
        self.counter += 1
        self.log(f"Received {self.counter} images")
        if self.counter == COUNTER_FREQUENCY:
            self.counter = 0
            # Capture start time
            
            send_package = pickle.dumps(msg.data)
            self.log(f"Sending package of size {len(send_package)}")
        
            # Send the image to the server and receive the response
            response = requests.post(f"http://{IP}:{PORT}/detect_objects", data=send_package)
            self.log(response.content)
            response = json.loads(response.content)
            
            if not response:
                return
            
            self.log(response[4])

            obstacle_info = {
                "message": "discover_obstacle",
                "id":  -1,
                "timestamp": time(),
                "label": "duckie",
                "duckieId": -1,
                "case": "Alarm! Too close. Stopping"
            }

            stop_cmd = f'rosparam set /{os.environ["VEHICLE_NAME"]}/kinematics_node/gain'
            if response[4] == 5:
                os.system(f'{stop_cmd} 0.0')
            elif response[4] != 0:
                obstacle_info["case"] = CASES[response[4]]
            else:
                os.system(f'{stop_cmd} 1.0')
                obstacle_info["message"] = "nothing's_on_the_way"
                obstacle_info["case"] = CASES[response[4]]

            reply = String()
            reply.data = json.dumps(obstacle_info)
            
            self.log(reply.data)
            self._detected_objs.publish(reply)


if __name__ == "__main__":
    node = ObjectDetectionNode()
    # spin forever
    rospy.spin()
