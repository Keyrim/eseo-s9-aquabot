import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from environment_interfaces.msg import BoatState

# Topics used by the estimator
in_topic_gps = '/wamv/sensors/gps/gps/fix'
in_topic_imu = '/wamv/sensors/imu/imu/data'

# Topics published by the estimator
out_topic_pos = '/boat/estimator/position'

class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')
        self.x = 0
        self.y = 0
        self.theta = 0
        self.previous_x = 0
        self.previous_y = 0
        self.vx = 0
        self.vy = 0
        self.previous_time_s = 0
        self.previous_time_ns = 0
        self.base_lat = None
        self.base_lon = None

        # Subs
        self.subscription_gps = self.create_subscription(
            NavSatFix,
            in_topic_gps,
            self.gps_callback,
            10)
        self.subscription_gps  # prevent unused variable warning
        self.subscription_imu = self.create_subscription(
            Imu,
            in_topic_imu,
            self.imu_callback,
            10)
        self.subscription_imu # prevent unused variable warning

        # Pubs
        self.publisher_pos = self.create_publisher(BoatState, out_topic_pos, 10)

    def gps_callback(self, msg: NavSatFix):
        latitude = msg.latitude
        longitude = msg.longitude
        # On récupère la position en x et y
        if self.base_lat is None and self.base_lon is None:
            self.base_lat, self.base_lon = latitude, longitude
            self.x, self.y = 0, 0
        else:
            self.x, self.y = self.convert_gps_to_xy(latitude, longitude)
        # On calcule la vitesse en x et y
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        dt = (sec - self.previous_time_s) + ((nsec - self.previous_time_ns) / 1e9)
        self.vx = (self.x - self.previous_x) / dt
        self.vy = (self.y - self.previous_y) / dt
        self.previous_time_s = sec
        self.previous_time_ns = nsec
        self.previous_x = self.x
        self.previous_y = self.y
        # Publish the state of the boat
        self.publish_state()

    def imu_callback(self, msg):
        # Get the orientation of the boat from the IMU
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.theta = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        # Publish the state of the boat
        self.publish_state()

    def publish_state(self):
        # Publish the state of the boat
        msg: BoatState = BoatState()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.vx = float(self.vx)
        msg.vy = float(self.vy)
        msg.theta = float(self.theta)
        self.publisher_pos.publish(msg)



    def convert_gps_to_xy(self, latitude, longitude):
        # Haversine distance calculation
        R = 6371000  # Earth radius in meters

        dLat = math.radians(latitude - self.base_lat)
        dLon = math.radians(longitude - self.base_lon)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.sin(dLon / 2) * math.sin(dLon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # Calculate x and y based on the distance and bearing
        y = math.sin(dLon) * math.cos(math.radians(latitude))
        x = math.cos(math.radians(self.base_lat)) * math.sin(math.radians(latitude)) - math.sin(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.cos(dLon)
        bearing = math.atan2(y, x)
        y = distance * math.cos(bearing)
        x = distance * math.sin(bearing)
        return x, y

def main(args=None):
    print("Estimator started")
    rclpy.init(args=args)
    estimator_node = Estimator()
    rclpy.spin(estimator_node)
    # Destroy the node explicitly
    estimator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
