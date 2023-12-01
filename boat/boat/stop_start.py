from std_msgs.msg import Int32

TOPIC_STOP_START = '/boat/stop_start'

class StartStopReceiver:
    def __init__(self, node):
        self.node = node
        self.order = 0 # 0: disabled, 1: enabled
        self.subscription = node.create_subscription(
            Int32, TOPIC_STOP_START, self.callback, 10)

    def callback(self, msg):
        self.order = msg.order
        self.node.start_stop_receiver()