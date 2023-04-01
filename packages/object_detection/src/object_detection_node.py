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


COUNTER_FREQUENCY = 1

IP = '192.168.0.43'
PORT = 8080


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

        self._detected_objs_led_publisher = rospy.Publisher(
            "~objects_detected_led_msg", String, queue_size=1, dt_topic_type=TopicType.DRIVER
        )
        self.flag_prev = 0
        

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

            cases = {1: 'nothing',
                     2: 'close',
                     3: 'nothing',
                     4: 'nothing',
                     5: 'detected',
                     0: 'nothing'}

            reply = String()

            obstacle_info = {
                            "type": "status_obstacle",
                            "data": {
                                "message": "nothing", 
                                "id": -1,
                                "timestamp": '',
                                "label": 'duckie',
                                "duckieId": -1,
                                "case": '' 
                                }
                            }

            if response[4] == 5:
                os.system(f'rosparam set /{os.environ["VEHICLE_NAME"]}/kinematics_node/gain 0.0')
                message = String()
                message.data = "obsticle"
                self._detected_objs_led_publisher.publish(message)
                self.flag_prev = 0
                obstacle_info["data"]["message"] = "discover_obstacle"
            elif response[4] == 0:
                if self.flag_prev < 3:
                    self.flag_prev += 1
                    obstacle_info["data"]["message"] = "discover_obstacle"
                elif self.flag_prev == 3:
                    self.flag_prev += 2
                    message = String()
                    message.data = "empty"

                    self._detected_objs_led_publisher.publish(message)
                    print('empty')
                    os.system(f'rosparam set /{os.environ["VEHICLE_NAME"]}/kinematics_node/gain 1.0')
                    obstacle_info["data"]["message"] = "remove_obstacle"
                
            else:
                if self.flag_prev < 3:
                    self.flag_prev += 1
                    obstacle_info["data"]["message"] = "discover_obstacle"
                elif self.flag_prev == 3:
                    self.flag_prev += 2
                    message = String()
                    message.data = "empty"
                    self._detected_objs_led_publisher.publish(message)
                    obstacle_info["data"]["message"] = "remove_obstacle"
                    print('empty')
                    os.system(f'rosparam set /{os.environ["VEHICLE_NAME"]}/kinematics_node/gain 1.0')
            reply.data = json.dumps(obstacle_info)
            self.log(reply.data)
            self._detected_objs.publish(reply)


if __name__ == "__main__":
    node = ObjectDetectionNode()
    # spin forever
    rospy.spin()
