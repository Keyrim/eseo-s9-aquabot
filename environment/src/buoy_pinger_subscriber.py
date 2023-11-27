from ros_gz_interfaces.msg import ParamVec
from src.shared_types import Source
from environment_interfaces.msg import BuoyInfo
from rclpy.node import Node

class BuoyPingerSubscriber(Node):
    def __init__(
        self,
    ):
        super().__init__("buoy_pinger_subscriber")
        self.subscription = self.create_subscription(
            ParamVec,
            "/wamv/sensors/acoustics/receiver/range_bearing",
            self.listener_callback,
            10,
        )
        self.buoy_position_publisher = self.create_publisher(
            BuoyInfo,
            "/environment/buoy_position",
            10
        )

    def listener_callback(self, msg: ParamVec):
        buoy_info = BuoyInfo()
        buoy_info.source = Source.ACOUSTIC
        buoy_info.is_found = True

        for param in msg.params:
            name = param.name
            if name == "bearing":
                buoy_info.angle = param.value.double_value
            elif name == "range":
                buoy_info.distance = param.value.double_value
        self.buoy_position_publisher.publish(buoy_info)
