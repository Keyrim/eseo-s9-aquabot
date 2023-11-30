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
        self.buoy_theta: float = 0.0
        self.buoy_range: float = 0.0
        self.buoy_theta_sum: float = 0.0
        self.buoy_range_sum: float = 0.0
        self.rx_counter: int = 1

    def listener_callback(self, msg: ParamVec):
        for param in msg.params:
            name = param.name
            if name == "bearing":
                # self.buoy_theta = param.value.double_value
                self.buoy_theta_sum += param.value.double_value
            elif name == "range":
                # self.buoy_range = param.value.double_value
                self.buoy_range_sum += param.value.double_value
        self.buoy_theta = self.buoy_theta_sum / self.rx_counter
        self.buoy_range = self.buoy_range_sum / self.rx_counter
        self.rx_counter += 1
        self.node.buoy_pinger_cb()