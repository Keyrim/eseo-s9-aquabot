from sensor_msgs.msg import LaserScan
import math
from rclpy.node import Node
from src.lidar_object_publisher import LidarObjectPublisher
from environment_interfaces.msg import LidarObject

class LidarSubscriber(Node):
    def __init__(
        self,
        lidar_object_publisher: LidarObjectPublisher,
    ):
        super().__init__("lidar_subscriber")
        self.subscription = self.create_subscription(
            LaserScan,
            "/wamv/sensors/lidars/lidar_wamv_sensor/scan",
            self.listener_callback,
            10,
        )
        self.lidar_object_publisher = lidar_object_publisher
    
    def listener_callback(self, msg: LaserScan):
        objectList: list = []
        pointList: list = []
        missing_points_limit_default = 5 # TODO adapt this value
        missing_points_counter = missing_points_limit_default
        new_object_found = False
        for i, distance in enumerate(msg.ranges):
            if not math.isinf(distance):
                missing_points_counter = missing_points_limit_default
                new_object_found = True
                # Calcul de l'angle correspondant à l'index i
                angle = i * msg.angle_increment
                # print("Point[", i, "] ; Angle = ",
                #       angle, "° ; distance = ", distance) # DEBUG
                pointList.append((angle, distance))
            elif missing_points_counter == 0 and new_object_found is True: # If consecutives points are inf, then we suppose the end of current object
                new_object_found = False
                objectList.append(pointList.copy())
                pointList.clear()
            else: # Decrement missing successives points
                missing_points_counter = missing_points_counter - 1

        for scanned_object in objectList:
            lidar_object = LidarObject()
            (angle_begin, distance_begin) = scanned_object[0]
            (angle_end, distance_end) = scanned_object[-1]
            lidar_object.angle_min = angle_begin
            lidar_object.angle_max = angle_end
            lidar_object.dist_begin = distance_begin
            lidar_object.dist_end = distance_end
            self.lidar_object_publisher.publish_lidar_object(lidar_object)
            # self.get_logger().info(f"angle {angle_begin:.2f} rad {distance_begin:.2f} m --------- {angle_end:.2f} rad {distance_end:.2f} m\n") # DEBUG
        objectList.clear()
        pointList.clear()