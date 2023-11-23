import rclpy
import threading
from rclpy.node import Node
from src.image_subscriber import ImageSubscriber
from src.threat_position_publisher import ThreatPositionPublisher
from src.buoy_position_publisher import BuoyPositionPublisher
from src.buoy_pinger_subscriber import BuoyPingerSubscriber
from src.lidar_object_publisher import LidarObjectPublisher
from src.lidar_subscriber import LidarSubscriber
from environment_interfaces.msg import BuoyInfo
from environment_interfaces.msg import ThreatInfo
from environment_interfaces.msg import LidarObject
from sensor_msgs.msg import NavSatFix

class Environment(Node):
    def __init__(self):
        super().__init__("environment")
        self.buoy_subscription = self.create_subscription(
            BuoyInfo,
            "/environment/buoy_position",
            self.buoy_listener_callback,
            10,
        )
        self.threat_subscription = self.create_subscription(
            ThreatInfo,
            "/environment/threat_position",
            self.threat_listener_callback,
            10,
        )
        self.lidar_object_subscription = self.create_subscription(
            LidarObject,
            "/environment/lidar_object",
            self.lidar_object_callback,
            10,
        )
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            "/wamv/sensors/gps/gps/fix", # TODO Subscribe to Theo filtered GPS topic
            self.gps_callback,
            10,
        )

    def buoy_listener_callback(self, msg: BuoyInfo):
        self.get_logger().info("BuoyInfo received") # DEBUG
        # TODO switch Source

    def threat_listener_callback(self, msg: ThreatInfo):
        self.get_logger().info("ThreatInfo received") # DEBUG
        # TODO switch Source

    def lidar_object_callback(self, msg: LidarObject):
        self.get_logger().info("LidarObject received") # DEBUG
        # TODO switch Source

    def gps_callback(self, msg: NavSatFix):
        self.get_logger().info("NavSatFix received") # DEBUG

    def run(self):
        self.get_logger().info("Environment module starts running...")
        sleeping_node = rclpy.create_node("waiting_node")

        # Initialize ROS nodes
        threat_position_publisher = ThreatPositionPublisher()
        buoy_position_publisher = BuoyPositionPublisher()
        lidar_object_publisher = LidarObjectPublisher()

        buoy_pinger_subscriber = BuoyPingerSubscriber(buoy_position_publisher)
        image_subscriber = ImageSubscriber(threat_position_publisher, buoy_position_publisher)
        lidar_subscriber = LidarSubscriber(lidar_object_publisher)

        # Create a multi-threaded node executor
        multi_threaded_executor = rclpy.executors.MultiThreadedExecutor()
        multi_threaded_executor.add_node(sleeping_node)
        multi_threaded_executor.add_node(threat_position_publisher)
        multi_threaded_executor.add_node(lidar_object_publisher)
        multi_threaded_executor.add_node(buoy_position_publisher)
        multi_threaded_executor.add_node(buoy_pinger_subscriber)
        multi_threaded_executor.add_node(image_subscriber)
        multi_threaded_executor.add_node(lidar_subscriber)
        multi_threaded_executor.add_node(self)

        # Spin in a separate thread
        executor_thread = threading.Thread(target=multi_threaded_executor.spin, daemon=True)
        executor_thread.start()
        rate = sleeping_node.create_rate(2)
        try:
            while rclpy.ok():
                rate.sleep()
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()
        executor_thread.join()

def main(args=None):
    rclpy.init()
    env = Environment()
    env.run()

if __name__ == '__main__':
    main()