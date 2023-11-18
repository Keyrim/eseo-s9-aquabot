from rclpy.node import Node
from environment_interfaces.msg import SearchedObjectInfo

class ThreatPositionPublisher(Node):
    def __init__(self):
        super().__init__('threat_position_publisher')
        self.publisher = self.create_publisher(SearchedObjectInfo, '/threat_position', 10)

    def publish_threat_position(self, threat_info: SearchedObjectInfo):
        self.publisher.publish(threat_info)