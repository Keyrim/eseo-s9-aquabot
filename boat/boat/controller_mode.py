from std_msgs.msg import Int32
from enum import Enum

TOPIC_CONTROLLER_MODE = '/boat/controller/mode'

class ControllerMode(Enum):
    DISABLED = 0
    ENABLED = 1


class ControllerModeReceiver:
    def __init__(self, node):
        self.node = node
        self.mode: ControllerMode = ControllerMode.DISABLED
        self.subscription = node.create_subscription(
            Int32, TOPIC_CONTROLLER_MODE, self.callback, 10)

    def callback(self, msg: Int32):
        self.mode = ControllerMode(msg.data)
        self.node.controller_mode_cb()


class ControllerModePublisher:
    def __init__(self, node):
        self.node = node
        self.mode = ControllerMode.DISABLED
        self.publisher = node.create_publisher(Int32, TOPIC_CONTROLLER_MODE, 10)

    def publish(self, mode: ControllerMode):
        msg = Int32()
        self.mode = mode
        msg.data = mode.value
        self.publisher.publish(msg)