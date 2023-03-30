#!/usr/bin/env python3
from __future__ import annotations

import rospy
import pickle
from time import perf_counter
import requests

# import torch
# from PIL import Image 

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage


COUNTER_FREQUENCY = 10

IP = '192.168.0.176'
PORT = 8000

# clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# clientSocket.connect((IP, PORT))

class ObjectDetectionNode(DTROS):
    def __init__(self) -> None:
        self.counter = 0
        super(ObjectDetectionNode, self).__init__(
            node_name="object_detection_node", node_type=NodeType.PERCEPTION
        )

        # self._model = torch.hub.load('ultralytics/yolov5', 'custom', path = 'model.pt')
        # self._model.load_state_dict(torch.load('model.pt')['model'].state_dict())

        self._img_sub = rospy.Subscriber(
            "~image", CompressedImage, self.cb_image, queue_size=1, buff_size="20MB"
        )

        # self._detected_objs = rospy.Publisher(
        #     "~objects_detected", String, queue_size=1, dt_topic_type=TopicType.DRIVER
        # )

    def cb_image(self, msg) -> None:
        # def sort_func(tensor: torch.Tensor) -> float:
        #     x1, y1, x2, y2, _, _ = tensor
        #     return (x2 - x1) * (y2 - y1)
        
        
        # dataFromServer = clientSocket.recv(1024)
        # print(dataFromServer.decode())
        
        self.counter += 1
        self.log(f"Received {self.counter} images")
        if self.counter == COUNTER_FREQUENCY:
            # Capture start time
            t_start = perf_counter()
            send_package = pickle.dumps(msg.data)
            self.log(f"Sending package of size {len(send_package)}")
            # Send the image to the server and receive the response
            response = requests.post(f"http://{IP}:{PORT}/detect_objects", data=send_package)
            self.counter = 0
            # Capture end time
            t_end = perf_counter()
            # Not sure what kind of objects will be sent (if it is an array of integers, it will be awful)
            self.log(f"Received response: {response} after {t_end - t_start} seconds")
            
            return


if __name__ == "__main__":
    node = ObjectDetectionNode()
    # spin forever
    rospy.spin()
