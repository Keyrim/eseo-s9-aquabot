import rclpy
from src.image_subscriber import ImageSubscriber
from src.threat_position_publisher import ThreatPositionPublisher
from src.buoy_position_publisher import BuoyPositionPublisher
from src.buoy_pinger_subscriber import BuoyPingerSubscriber
import threading

def main():
    rclpy.init()
    print("Environment module")
    sleeping_node = rclpy.create_node('waiting_node')

    # Initialize ROS nodes
    threat_position_publisher = ThreatPositionPublisher()
    buoy_position_publisher = BuoyPositionPublisher()

    buoy_pinger_subscriber = BuoyPingerSubscriber(buoy_position_publisher)
    image_subscriber = ImageSubscriber(threat_position_publisher, buoy_position_publisher)

    # Create a multi-threaded node executor
    multi_threaded_executor = rclpy.executors.MultiThreadedExecutor()  
    multi_threaded_executor.add_node(sleeping_node)
    multi_threaded_executor.add_node(threat_position_publisher)
    multi_threaded_executor.add_node(buoy_position_publisher)
    multi_threaded_executor.add_node(buoy_pinger_subscriber)
    multi_threaded_executor.add_node(image_subscriber)

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

    # # Start ROS nodes
    # rclpy.spin(threat_position_publisher)
    # print("threat_position_publisher started")
    # rclpy.spin(buoy_position_publisher)
    # print("buoy_position_publisher started")

    # rclpy.spin(image_subscriber)
    # print("image_subscriber started")
    # rclpy.spin(buoy_pinger_subscriber)
    # print("buoy_pinger_subscriber started")

    # # Free resources
    # image_subscriber.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
