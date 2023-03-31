#!/usr/bin/env python3
import time
import rospy
from dataclasses import dataclass

from std_msgs.msg import String, Float32MultiArray
from duckietown.dtros import DTROS, NodeType, TopicType

from datetime import datetime
from json import jsonify
from enum import Enum

from typing import List, Any


class EventType(Enum):
    ONE = 1
    TWO = 2
    THREE = 3


@dataclass
class Event:
    type_: EventType
    start_time: datetime.datetime  # fix
    end_time: datetime.datetime  # fix
    building_x: float
    building_y: float

    def __init__(self, type_, start_time, end_time, x, y):
        self.type_: EventType = type_

        self.start_time = start_time
        self.end_time = end_time

        self.building_x = x
        self.building_y = y


COLORS = {
    EventType.ONE: [0.5, 0.4, 0.1],
    EventType.TWO: [0.5, 0.2, 0],
    EventType.THREE: [1, 1, 0.1],
}


class ContextAwarenessNode(DTROS):
    """Node for controlling LEDs.

    Publishes to the `~led_pattern` topic. If the absence of the FIFOs this should be remapped to the
     `led_driver_node/led_pattern` topic. The desired behavior is specified by
    the LED index (Duckiebots and watchtowers have multiple of these) and a pattern.
    A pattern is a combination of colors and blinking frequency.

    Duckiebots have 5 LEDs that are indexed and positioned as following:

    +------------------+------------------------------------------+
    | Index            | Position (rel. to direction of movement) |
    +==================+==========================================+
    | 0                | Front left                               |
    +------------------+------------------------------------------+
    | 1                | Rear left                                |
    +------------------+------------------------------------------+
    | 2                | Top / Front middle                       |
    +------------------+------------------------------------------+
    | 3                | Rear right                               |
    +------------------+------------------------------------------+
    | 4                | Front right                              |
    +------------------+------------------------------------------+

    A pattern is specified via 5 parameters:

    - its name

    - frequency: blinking frequency in Hz, should be set to 0 for a solid (non-blinking) behavior

    - color_list: a list of 5 colour names (see below), one for each LED ordered as above, or a single string with
      a single color name that would be applied to all LEDs

    - frequency_mask: a list of 5 binary flags (0 or 1) that specify which of the LEDs should be blinking,
      used only if the frequency is not 0. The LEDs with the flag set to 0, will maintain their solid color.

    The defaut patterns are defined in the `LED_protocol.yaml` configuration file for the node.

    Currently supported colors are: `green`, `red`, `blue`, `white`, `yellow`, `purple`, `cyan`,
    `pink`, `switchedoff`. More colors can be defined in the node's configuration file.

    Examples:

        To change the pattern to one of the predefined patterns (you can see them using `rosparam list`)
        use a variant of the following::

            rosservice call /HOSTNAME/led_emitter_node/set_pattern "pattern_name: {data: RED}"

        Other pre-defined patterns you can use are: `WHITE`, `GREEN`, `BLUE`, `LIGHT_OFF`, `CAR_SIGNAL_PRIORITY`,
        `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`,
        `CAR_DRIVING`.

        To add a custom pattern and switch to it use a variant of the following::

            rosservice call /HOSTNAME/led_emitter_node/set_custom_pattern "pattern: {color_list: ['green','yellow','pink','orange','blue'], color_mask: [1,1,1,1,1], frequency: 1.0, frequency_mask: [1,0,1,0,1]}"


    Configuration:
        ~LED_protocol (nested dictionary): Nested dictionary that describes the LED protocols (patterns). The
            default can be seen in the `LED_protocol.yaml` configuration file for the node.
        ~LED_scale (:obj:`float`): A scaling factor (between 0 and 1) that is applied to the colors in order
            to reduce the overall LED brightness, default is 0.8.
        ~channel_order (:obj:`str`): A string that controls the order in which the 3 color channels should be
            communicated to the LEDs. Should be one of 'RGB', `RBG`, `GBR`, `GRB`, `BGR`, `BRG`. Typically
            for a duckiebot this should be the default `RGB` and for traffic lights should be `GRB`, default is `RGB`.

    Publishers:
        ~led_pattern (:obj:`LEDPattern` message): Publishes the 5 LED values to be set by
        the LED driver

    Services:
        ~set_custom_pattern: Allows setting a custom protocol. Will be named `custom`. See an example of a call
            in :obj:`srvSetCustomLEDPattern`.

            input:

                pattern (:obj:`LEDPattern` message): The desired new LEDPattern

        ~set_pattern: Switch to a different pattern protocol.

            input:

                pattern_name (:obj:`String` message): The new pattern name, should match one of the patterns in
                   the `LED_protocol` parameter (or be `custom` if a custom pattern has been defined via a call to
                   the `~change_led` service.

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(ContextAwarenessNode, self).__init__(
            node_name=node_name, node_type=NodeType.DRIVER)

        # Subscribers
        self.sub_msg = rospy.Subscriber(
            "~context_message", String, self.parse_event, queue_size=1, buff_size="20MB"
        )

        self.sub_coord = rospy.Subscriber(
            "~coordinates", String, self.parse_coord, queue_size=1, buff_size="20MB"
        )

        # Publishers
        self.pub_led_color = rospy.Publisher(
            "~color", Float32MultiArray, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # Class members
        self.events: List[Event] = []

    def _clean_up_events(self, cur_time):
        """erases passed events"""
        new_lst = []
        for event in self.events:
            if event.end_time >= cur_time:
                new_lst.append(event)
        self.events = new_lst

    """
    msg_sample:
    {"type": "plan_construction_site",
    "data": {
         "id": UUID,
         "coordinates": [{ # More than one tile
            "x": DOUBLE,
            "y": DOUBLE,
            "x_abs": DOUBLE,
            "y_abs": DOUBLE,
            "quadrant": INT[]
          }],
         "startDateTime": DATETIME,
        "endDateTime": DATETIME,
        "maximumSpeed": DOUBLE,
        "traffic_lights": {
            "id1": UUID,
            "id2": UUID
        }
    }
    """

    def parse_event(self, msg):
        """parse new event"""
        msg = jsonify(msg)  # convert to dict ?????

        type_to_enum = {
            # TODO
        }

        match msg["type"]:
            case "null":
                ...
            case "one" | "two" | "three":
                data = msg["data"]
                type_ = type_to_enum(msg["type"])
                # id = data[id]

                start_time = datetime.strptime(data["start_time"], '%H:%M:%S')
                end_time = datetime.strptime(data["end_time"], '%H:%M:%S')

                coordinates = data["coordinate"]

                x = coordinates["x"]
                y = coordinates["y"]
                self.events.append(Event(type_, start_time, end_time, x, y))
            case _:
                return

    def parse_coord(self, msg):
        """receives message coordinates and responds to them"""
        msg = jsonify(msg)  # not sure if works tho
        x, y = msg["x"], msg["y"]
        EPS = 1e2

        cur_time = datetime.strptime(msg["time"])

        self._clean_up_events(cur_time)

        req = Float32MultiArray()
        for event in self.events:
            cur_dist = ((x - event.building_x) ** 2 +
                        (y - event.building_y) ** 2) ** (1 / 2)
            if cur_dist < EPS and event.start_time <= cur_time <= event.end_time:
                req.data = COLORS[event.type_]
                self.pub_led_color.publish(req)
                # match event.type_:
                #     case EventType.ONE:
                #         ...
                #     case EventType.TWO:
                #         ...
                #     case EventType.THREE:
                #         ...
                #     case _:
                #         ...


if __name__ == "__main__":
    # Create the LEDEmitterNode object
    led_emitter_node = ContextAwarenessNode(node_name="context_awareness")
    # Keep it spinning to keep the node alive
    rospy.spin()
