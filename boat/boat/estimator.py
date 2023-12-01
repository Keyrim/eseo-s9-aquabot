import rclpy
import math
from rclpy.node import Node
from boat.gps_handler import GPSHandler
from boat.imu_handler import IMUHandler
from boat.boat_state import BoatStatePublisher


class Estimator(Node):
    def __init__(self):
        super().__init__("estimator")
        self.gps_handler = GPSHandler(self)
        self.imu_handler = IMUHandler(self)
        self.boat_state_publisher = BoatStatePublisher(self)
        self.is_base_lat_lon_set = False

    def publish_state(self):
        # log current x y position with 0 decimals and angle converted from rad to deg
        self.get_logger().info('x: %f, y: %f, theta: %f' % (self.boat_state_publisher.x, self.boat_state_publisher.y, self.boat_state_publisher.theta*180/math.pi))
        if self.is_base_lat_lon_set:
            self.boat_state_publisher.publish()

    def gps_handler_cb(self):
        self.boat_state_publisher.update_pos_and_speed(
            self.gps_handler.x,
            self.gps_handler.y,
            self.gps_handler.vx,
            self.gps_handler.vy,
        )
        if not self.is_base_lat_lon_set:
            self.boat_state_publisher.set_base_lat_lon(
                self.gps_handler.base_lat,
                self.gps_handler.base_lon,
            )
            self.is_base_lat_lon_set = True
        self.publish_state()

    def imu_handler_cb(self):
        self.boat_state_publisher.update_orientation(self.imu_handler.theta)
        self.publish_state()


def main(args=None):
    print("Estimator started")
    rclpy.init(args=args)
    estimator_node = Estimator()
    rclpy.spin(estimator_node)
    estimator_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
