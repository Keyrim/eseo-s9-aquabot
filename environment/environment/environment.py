import rclpy
from environment.image_subscriber import ImageSubscriber
from environment.threat_position_publisher import ThreatPositionPublisher
from environment.buoy_position_publisher import BuoyPositionPublisher

def main():
    rclpy.init()
    print("Environment module")

    # Initialize ROS nodes
    threat_position_publisher = ThreatPositionPublisher()
    buoy_position_publisher = BuoyPositionPublisher()

    image_subscriber = ImageSubscriber(threat_position_publisher, buoy_position_publisher)

     # Start ROS nodes
    rclpy.spin(image_subscriber)
    rclpy.spin(threat_position_publisher)

    # Free resources
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
