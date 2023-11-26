from ros_gz_interfaces.msg import ParamVec
from src.shared_types import Source
from src.buoy_position_publisher import BuoyPositionPublisher
from environment_interfaces.msg import BuoyInfo
from rclpy.node import Node
import math

class BuoyPingerSubscriber(Node):
    def __init__(
        self,
        buoy_position_publisher: BuoyPositionPublisher,
    ):
        super().__init__("buoy_pinger_subscriber")
        self.subscription = self.create_subscription(
            ParamVec,
            "/wamv/sensors/acoustics/receiver/range_bearing",
            self.listener_callback,
            10,
        )
        self.buoy_position_publisher = buoy_position_publisher

    def listener_callback(self, msg: ParamVec):
        buoy_info = BuoyInfo()
        buoy_info.source = Source.ACOUSTIC
        buoy_info.is_found = True

        for param in msg.params:
            name = param.name
            if name == 'bearing':
                buoy_info.angle = param.value.double_value
            elif name == 'range':
                buoy_info.distance = param.value.double_value
        self.buoy_position_publisher.publish_buoy_position(buoy_info)