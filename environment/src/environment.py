import rclpy
import math
import threading
from geometry_msgs.msg import Point
from rclpy.node import Node
from environment_interfaces.msg import BoatState
from src.image_subscriber import ImageSubscriber
from src.lidar_subscriber import LidarSubscriber
from environment_interfaces.msg import BuoyInfo
from environment_interfaces.msg import ThreatInfo
from environment_interfaces.msg import LidarCluster
from src.shared_types import Source
from boat.boat_state import BoatStateReceiver
from src.buoy_pinger_subscriber import BuoyPingerSubscriber

class Environment(Node):
    def __init__(self):
        super().__init__("environment")
        self.threat_subscription = self.create_subscription(
            ThreatInfo,
            "/environment/threat_position",
            self.threat_listener_callback,
            10,
        )
        self.lidar_object_subscription = self.create_subscription(
            LidarCluster,
            "/environment/lidar_object",
            self.lidar_cluster_callback,
            10,
        )
        self.buoy_pos_publisher = self.create_publisher(
            Point,
            "/environment/buoy_pos",
            10
        )
        self.boat_state_receiver = BoatStateReceiver(self)
        self.buoy_pinger_subscriber = BuoyPingerSubscriber(self)
        # Class attributes
        self.usv_x = 0.0
        self.usv_y = 0.0
        self.usv_theta = 0.0
        self.buoy_pos_pinger = Point() # Set by BuoyPingerSubscriber
        self.base_lat = 48.046300000000 # TODO remove
        self.base_lon = -4.976320000000 # TODO remove
        self.buoy_info_lidar = BuoyInfo()
        self.buoy_pos = {Source.ACOUSTIC: Point(), Source.LIDAR: Point(), Source.CAMERA: Point()}

    def lidar_cluster_callback(self, msg: LidarCluster):
        pass
        # TODO switch Source

    def threat_listener_callback(self, msg: ThreatInfo):
        pass
        # TODO switch Source

    def boat_state_receiver_cb(self): # called by BoatStateReceiver callback
        #self.get_logger().info("BoatState received") # DEBUG
        self.usv_x = self.boat_state_receiver.x
        self.usv_y = self.boat_state_receiver.y
        self.usv_theta = self.boat_state_receiver.theta

    def buoy_pinger_cb(self):
        buoy_range = self.buoy_pinger_subscriber.buoy_range
        buoy_theta = self.buoy_pinger_subscriber.buoy_theta
        self.buoy_pos[Source.ACOUSTIC].x = self.usv_x + (buoy_range * math.cos(buoy_theta + self.usv_theta)) # Consider USV orientation
        self.buoy_pos[Source.ACOUSTIC].y = self.usv_y + (buoy_range * math.sin(buoy_theta + self.usv_theta)) # Consider USV orientation
        self.get_logger().info(f"[buoy] pos ({self.buoy_pos[Source.ACOUSTIC].x:.2f};{self.buoy_pos[Source.ACOUSTIC].y:.2f}) ; pinger {buoy_theta:.2f} ; usv {self.usv_theta:.2f}")
        self.buoy_pos_publisher.publish(self.buoy_pos[Source.ACOUSTIC])

    def run(self):
        #self.get_logger().info("Environment module starts running...")
        sleeping_node = rclpy.create_node("waiting_node")

        # Initialize ROS nodes
        image_subscriber = ImageSubscriber()
        lidar_subscriber = LidarSubscriber()

        # Create a multi-threaded node executor
        multi_threaded_executor = rclpy.executors.MultiThreadedExecutor()
        multi_threaded_executor.add_node(sleeping_node)
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

    # def convert_gps_to_xy(self, latitude, longitude):
    #     # Haversine distance calculation
    #     R = 6371000  # Earth radius in meters

    #     dLat = math.radians(latitude - self.base_lat)
    #     dLon = math.radians(longitude - self.base_lon)
    #     a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.sin(dLon / 2) * math.sin(dLon / 2)
    #     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    #     distance = R * c

    #     # Calculate x and y based on the distance and bearing
    #     y = math.sin(dLon) * math.cos(math.radians(latitude))
    #     x = math.cos(math.radians(self.base_lat)) * math.sin(math.radians(latitude)) - math.sin(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.cos(dLon)
    #     bearing = math.atan2(y, x)
    #     y = distance * math.cos(bearing)
    #     x = distance * math.sin(bearing)
    #     return x, y
    
def main(args=None):
    rclpy.init()
    env = Environment()
    env.run()

if __name__ == '__main__':
    main()