import rclpy
import math
import threading
from geometry_msgs.msg import Point
from rclpy.node import Node
from src.image_subscriber import ImageSubscriber
from src.buoy_pinger_subscriber import BuoyPingerSubscriber
from src.lidar_subscriber import LidarSubscriber
from environment_interfaces.msg import BuoyInfo
from environment_interfaces.msg import ThreatInfo
from environment_interfaces.msg import LidarCluster
from environment_interfaces.msg import BoatState
from src.shared_types import Source

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
            LidarCluster,
            "/environment/lidar_object",
            self.lidar_object_callback,
            10,
        )
        self.gps_subscription = self.create_subscription(
            BoatState,
            "/boat/estimator/position", # TODO Subscribe to Theo filtered GPS topic
            self.gps_callback,
            10,
        )

        self.buoy_pos_publisher = self.create_publisher(
            Point,
            "/environment/buoy_pos",
            10
        )
        # Class attributes
        self.usv_x = 0
        self.usv_y = 0
        self.usv_theta = 0
        self.buoy_info_lidar = BuoyInfo()
        self.buoy_info_pinger = BuoyInfo()
        self.buoy_pos = Point()

    def buoy_listener_callback(self, msg: BuoyInfo):
        #self.get_logger().info("BuoyInfo received") # DEBUG
        if msg.source == Source.ACOUSTIC:
            self.buoy_info_pinger.distance = msg.distance
            self.buoy_info_pinger.angle = msg.angle
            self.buoy_info_pinger.is_found = msg.is_found

            buoy_pos = Point()
            buoy_pos.y = self.usv_y + (msg.distance * math.sin(msg.angle + self.usv_theta)) # Consider USV orientation
            buoy_pos.x = self.usv_x + (msg.distance * math.cos(msg.angle + self.usv_theta)) # Consider USV orientation
            self.get_logger().info(f"[buoy] pos ({buoy_pos.x:.2f};{buoy_pos.y:.2f}) ; pinger {msg.angle:.2f} ; usv {self.usv_theta:.2f} ")
            self.buoy_pos_publisher.publish(buoy_pos)
    
    def lidar_object_callback(self, LidarCluster):
        a = 0
        #self.get_logger().info("LidarCluster received") # DEBUG
        # TODO switch Source

    def threat_listener_callback(self, msg: ThreatInfo):
        a = 0
        #self.get_logger().info("ThreatInfo received") # DEBUG
        # TODO switch Source

    def gps_callback(self, msg: BoatState):
        #self.get_logger().info("BoatState received") # DEBUG
        self.usv_x = msg.x
        self.usv_y = msg.y
        self.usv_theta = msg.theta

    def run(self):
        #self.get_logger().info("Environment module starts running...")
        sleeping_node = rclpy.create_node("waiting_node")

        # Initialize ROS nodes
        buoy_pinger_subscriber = BuoyPingerSubscriber()
        image_subscriber = ImageSubscriber()
        lidar_subscriber = LidarSubscriber()

        # Create a multi-threaded node executor
        multi_threaded_executor = rclpy.executors.MultiThreadedExecutor()
        multi_threaded_executor.add_node(sleeping_node)
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
        print("Shutting down...")
        rclpy.shutdown()
        executor_thread.join()

def main(args=None):
    rclpy.init()
    env = Environment()
    env.run()

if __name__ == '__main__':
    main()