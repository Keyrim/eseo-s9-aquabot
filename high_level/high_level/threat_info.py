from environment_interfaces.msg import ThreatInfo

TOPIC_THREAT_INFO = "/environment/threat_info"


class ThreatInfoReceiver:
    def __init__(self, node):
        self.node = node
        self.x = 0
        self.y = 0
        self.subscription = node.create_subscription(
            ThreatInfo, TOPIC_THREAT_INFO, self.callback, 10)

    def callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.node.threat_info_cb()

    def get_pos(self):
        return (self.x, self.y)