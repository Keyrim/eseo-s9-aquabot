import rclpy
from rclpy.node import Node
from boat.GPSHandler import GPSHandler
from boat.boat.IMUHandler import IMUHandler
from boat.BoatState import BoatStatePublisher

class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')
        self.gps_handler = GPSHandler(self)
        self.imu_handler = IMUHandler(self)
        self.boat_state_publisher = BoatStatePublisher(self)

    def publish_state(self):
        # Utiliser les données de gps_handler et imu_handler pour publier l'état
        msg = BoatState()
        msg.x = float(self.gps_handler.x)
        msg.y = float(self.gps_handler.y)
        msg.theta = float(self.imu_handler.theta)
        self.publisher_pos.publish(msg)

def main(args=None):
    print("Estimator started")
    rclpy.init(args=args)
    estimator_node = Estimator()
    rclpy.spin(estimator_node)
    estimator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
