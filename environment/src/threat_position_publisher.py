from rclpy.node import Node
from environment_interfaces.msg import ThreatInfo

class ThreatPositionPublisher(Node):
    def __init__(self):
        super().__init__('threat_position_publisher')
        self.publisher = self.create_publisher(ThreatInfo, "/environment/threat_position", 10)

    def publish_threat_position(self, threat_info: ThreatInfo):
        self.publisher.publish(threat_info)