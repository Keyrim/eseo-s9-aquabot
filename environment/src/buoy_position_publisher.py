from rclpy.node import Node
from environment_interfaces.msg import SearchedObjectInfo

class BuoyPositionPublisher(Node):
    def __init__(self):
        super().__init__("buoy_position_publisher")
        self.publisher = self.create_publisher(SearchedObjectInfo, "/buoy_position", 10)

    def publish_buoy_position(self, buoy_info: SearchedObjectInfo):
        self.publisher.publish(buoy_info)
