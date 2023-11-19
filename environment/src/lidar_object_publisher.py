from rclpy.node import Node
from environment_interfaces.msg import LidarObject

class LidarObjectPublisher(Node):
    def __init__(self):
        super().__init__("lidar_object_publisher")
        self.publisher = self.create_publisher(LidarObject, "/environment/lidar_object", 10)

    def publish_lidar_object(self, lidar_object: LidarObject):
        self.publisher.publish(lidar_object)
