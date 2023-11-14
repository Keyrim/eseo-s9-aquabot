import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class Estimator(Node):

    def __init__(self):
        super().__init__('estimator')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10)
        self.subscription  # prevent unused variable warning

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        self.get_logger().info(f'GPS Data - Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}')


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
