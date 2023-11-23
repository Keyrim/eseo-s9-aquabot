import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from environment_interfaces.msg import BoatState

# Topics used by the estimator
in_topic_gps = '/wamv/sensors/gps/gps/fix'

# Topics published by the estimator
out_topic_pos = '/boat/estimator/position'

class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')
        self.x = 0
        self.y = 0
        self.base_lat = None
        self.base_lon = None

        # Subs
        self.subscription_gps = self.create_subscription(
            NavSatFix,
            in_topic_gps,
            self.gps_callback,
            10)
        self.subscription_gps  # prevent unused variable warning
        # Pubs
        self.publisher_pos = self.create_publisher(BoatState, out_topic_pos, 10)

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude

        if self.base_lat is None and self.base_lon is None:
            self.base_lat, self.base_lon = latitude, longitude
            self.x, self.y = 0, 0
        else:
            self.x, self.y = self.convert_gps_to_xy(latitude, longitude)
        # Affichage de la position en x et y avec 3 décimakes
        self.get_logger().info('x: %.3f, y: %.3f' % (self.x, self.y))
        # Publication de la position en x et y
        msg = BoatState()
        msg.x = float(self.x)
        msg.y = float(self.y)
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
        y = - distance * math.cos(bearing)
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
