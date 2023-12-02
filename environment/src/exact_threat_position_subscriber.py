from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class ExactThreatPositionSubscriber(Node):
    def __init__(
        self,
        node: Node
    ):
        self.node = node
        self.allies_subscription = self.node.create_subscription(
            PoseStamped,
            "/wamv/ais_sensor/ennemy_position",
            self.exact_threat_position_cb,
            10,
        )
        self.exact_threat_lon = 0
        self.exact_threat_lat = 0

    def exact_threat_position_cb(self, msg: PoseStamped):
        self.exact_threat_lon = msg.pose.position.x
        self.exact_threat_lat = msg.pose.position.y
        self.node.exact_threat_pos_cb()