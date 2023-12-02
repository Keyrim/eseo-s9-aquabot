from std_msgs.msg import UInt32
from enum import Enum



TOPIC_CURRENT_STATE = "/vrx/patrolandfollow/current_phase"


class Phase(Enum):
    INIT = 0
    BUOY = 1
    PATROL = 2
    PURSUIT = 3

class StateTrackerReceiver:
    def __init__(self, node):
        self.node = node
        self.phase = Phase.INIT
        self.subscription = node.create_subscription(
            UInt32, TOPIC_CURRENT_STATE, self.callback, 10)

    def callback(self, msg):
        self.phase = Phase(msg.data)
        # Publish the state of the boat
        self.node.state_tracker_cb()

    def get_phase(self):
        return self.phase