from ros_gz_interfaces.msg import ParamVec
from rclpy.node import Node

class BuoyPingerSubscriber():
    def __init__(
        self,
        node: Node
    ):
        self.node = node
        self.subscription = self.node.create_subscription(
            ParamVec,
            "/wamv/sensors/acoustics/receiver/range_bearing",
            self.listener_callback,
            10,
        )
        self.buoy_theta = 0.0
        self.buoy_range = 0.0

    def listener_callback(self, msg: ParamVec):
        for param in msg.params:
            name = param.name
            if name == "bearing":
                self.buoy_theta = param.value.double_value
            elif name == "range":
                self.buoy_range = param.value.double_value
        self.node.buoy_pinger_cb()