import rclpy
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

    def publish_state(self):
        self.boat_state_publisher.publish()

    def gps_handler_cb(self):
        self.boat_state_publisher.update_pos_and_speed(
            self.gps_handler.x,
            self.gps_handler.y,
            self.gps_handler.vx,
            self.gps_handler.vy,
        )
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
