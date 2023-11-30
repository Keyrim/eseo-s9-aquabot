import rclpy
import math
import threading
from geometry_msgs.msg import Point
from rclpy.node import Node
from src.image_subscriber import ImageSubscriber
from src.lidar_subscriber import LidarSubscriber
from environment_interfaces.msg import BuoyInfo
from environment_interfaces.msg import ThreatInfo
from environment_interfaces.msg import LidarCluster
from src.shared_types import Source
from boat.boat_state import BoatStateReceiver
from src.buoy_pinger_subscriber import BuoyPingerSubscriber
from src.allies_subscriber import AlliesPositionSubscriber
import numpy as np
import time
import matplotlib.pyplot as plt

class CameraObject:
    def __init__(self):
        self.in_is_found = False  # Is the threat found by the camera ?
        self.in_rel_theta = 0.0  # Relative angle of the threat estimated by the camera
        self.out_x = 0.0  # X position of the threat computed with parametric equation
        self.out_y = 0.0  # Y position of the threat computed with parametric equation
        self.out_range = 0.0
        self.out_abs_theta = 0.0


class Environment(Node):
    def __init__(self):
        super().__init__("environment")
        self.lidar_object_subscription = self.create_subscription(
            LidarCluster,
            "/environment/lidar_object",
            self.lidar_cluster_callback,
            10,
        )
        self.buoy_pos_publisher = self.create_publisher(
            Point, "/environment/buoy_pos", 10
        )
        # Subscribers
        self.threat_image_subscriber = ImageSubscriber(self)
        self.boat_state_receiver = BoatStateReceiver(self)
        self.buoy_pinger_subscriber = BuoyPingerSubscriber(self)
        self.allies_position_subscriber = AlliesPositionSubscriber(self)
        # Class attributes
        # Camera management
        self.threat_camera = CameraObject()
        self.max_range_usv_threat = 1000  # [m] Distance maximale considérée pour la recherche de la menace entre l'USV et la menace
        self.max_range_origin_threat = 1000  # [m] Distance maximale considérée pour la recherche de la menace entre l'origine et la menace
        self.usv_threat_distances = np.arange(self.max_range_usv_threat)
        self.origin_threat_distances = np.arange(self.max_range_origin_threat)
        self.usv_x = 0.0
        self.usv_y = 0.0
        self.usv_theta = 0.0
        self.allies_pos_xy = []
        self.buoy_pos_pinger = Point()  # Set by BuoyPingerSubscriber
        self.base_lat = 48.046300000000  # TODO read it from estimator service
        self.base_lon = -4.976320000000  # TODO read it from estimator service
        self.buoy_info_lidar = BuoyInfo()
        self.buoy_pos_xy = {
            Source.ACOUSTIC: Point(),
            Source.LIDAR: Point(),
            Source.CAMERA: Point(),
        }

    def lidar_cluster_callback(self, msg: LidarCluster):
        pass
        # TODO switch Source

    def image_listener_threat_cb(self):
        self.threat_camera.in_is_found = self.threat_image_subscriber.is_threat_found
        self.threat_camera.in_rel_theta = self.threat_image_subscriber.threat_theta
        # if self.threat_camera.in_is_found and math.isfinite:
            # self.estimate_threat_position()

    def boat_state_receiver_cb(self):  # called by BoatStateReceiver callback
        # self.get_logger().info("BoatState received") # DEBUG
        self.usv_x = self.boat_state_receiver.x
        self.usv_y = self.boat_state_receiver.y
        self.usv_theta = self.boat_state_receiver.theta

    def buoy_pinger_cb(self):
        buoy_range = self.buoy_pinger_subscriber.buoy_range
        buoy_theta = self.buoy_pinger_subscriber.buoy_theta
        self.buoy_pos_xy[Source.ACOUSTIC].x = self.usv_x + (
            buoy_range * math.cos(buoy_theta + self.usv_theta)
        )  # Consider USV orientation
        self.buoy_pos_xy[Source.ACOUSTIC].y = self.usv_y + (
            buoy_range * math.sin(buoy_theta + self.usv_theta)
        )  # Consider USV orientation
        self.get_logger().info(
            f"[buoy] pos ({self.buoy_pos_xy[Source.ACOUSTIC].x:.2f};{self.buoy_pos_xy[Source.ACOUSTIC].y:.2f}) ; pinger {buoy_theta:.2f} ; usv {self.usv_theta:.2f}"
        )
        self.buoy_pos_publisher.publish(self.buoy_pos_xy[Source.ACOUSTIC])

    def allies_position_cb(self):
        init_flag = len(self.allies_pos_xy) == 0
        count = 0
        for ally_pos in self.allies_position_subscriber.allies_pos:
            if init_flag is True:
                self.allies_pos_xy.append(
                    self.convert_gps_to_xy(ally_pos[0], ally_pos[1])
                )
            else:
                self.allies_pos_xy[count] = self.convert_gps_to_xy(
                    ally_pos[0], ally_pos[1]
                )
                # print(f"Ally[{count} : ({self.allies_pos_xy[count][0]:.2f};{self.allies_pos_xy[count][1]:.2f})]") # DEBUG
            count += 1

    def run(self):
        # self.get_logger().info("Environment module starts running...")
        sleeping_node = rclpy.create_node("waiting_node")

        # Initialize ROS nodes
        lidar_subscriber = LidarSubscriber()

        # Create a multi-threaded node executor
        multi_threaded_executor = rclpy.executors.MultiThreadedExecutor()
        multi_threaded_executor.add_node(sleeping_node)
        multi_threaded_executor.add_node(lidar_subscriber)
        multi_threaded_executor.add_node(self)

        # Spin in a separate thread
        executor_thread = threading.Thread(
            target=multi_threaded_executor.spin, daemon=True
        )
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

    def convert_gps_to_xy(self, latitude, longitude):
        # Haversine distance calculation
        R = 6371000  # Earth radius in meters

        dLat = math.radians(latitude - self.base_lat)
        dLon = math.radians(longitude - self.base_lon)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(
            math.radians(self.base_lat)
        ) * math.cos(math.radians(latitude)) * math.sin(dLon / 2) * math.sin(dLon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # Calculate x and y based on the distance and bearing
        y = math.sin(dLon) * math.cos(math.radians(latitude))
        x = math.cos(math.radians(self.base_lat)) * math.sin(
            math.radians(latitude)
        ) - math.sin(math.radians(self.base_lat)) * math.cos(
            math.radians(latitude)
        ) * math.cos(
            dLon
        )
        bearing = math.atan2(y, x)
        y = distance * math.cos(bearing)
        x = distance * math.sin(bearing)
        return x, y

    def estimate_threat_position(self):
        t0 = time.time()  # DEBUG
        # Calcul des coordonnées des points des demi-droites en utilisant les équations paramétriques
        self.threat_camera.out_abs_theta = (
            self.usv_theta + self.threat_camera.in_rel_theta
        )
        usv_threat_line = np.column_stack(
            (
                self.usv_x
                + self.usv_threat_distances * np.cos(self.threat_camera.out_abs_theta),
                self.usv_y
                + self.usv_threat_distances * np.sin(self.threat_camera.out_abs_theta),
            )
        )
        origine_threat_line = np.column_stack(
            (
                self.origin_threat_distances * np.cos(self.usv_theta),
                self.origin_threat_distances * np.sin(self.usv_theta),
            )
        )
        distances_matrix = np.linalg.norm(
            usv_threat_line[:, None] - origine_threat_line, axis=2
        )

        # Recherche des deux points les plus proches
        idx_min = np.unravel_index(np.argmin(distances_matrix), distances_matrix.shape)
        closest_points = (usv_threat_line[idx_min[0]], origine_threat_line[idx_min[1]])
        print(f"closest_points: {closest_points}")  # DEBUG
        min_distance = math.hypot(
                closest_points[0][0] - closest_points[1][0],
                closest_points[0][1] - closest_points[1][1],
            )

        if (
            min_distance < 1
        ):  # Si la distance entre les deux points les plus proches est inférieure à 2 mètres, on considère le point moyen entre les deux points comme la position de la menace
            # Calcul du point moyen entre les deux points les plus proches pour connaître la position de la menace
            self.threat_camera.out_x = (closest_points[0][0] + closest_points[1][0]) / 2
            self.threat_camera.out_y = (closest_points[0][1] + closest_points[1][1]) / 2
            # Calcul de la distance USV-menace
            self.threat_camera.out_range = math.hypot(
                self.threat_camera.out_x - self.usv_x,
                self.threat_camera.out_y - self.usv_y,
            )
            t1 = time.time()  # DEBUG
            # self.get_logger().info(
            #     f"[Camera] Threat location estimation [took {t1 - t0:.6f} sec]: ({self.threat_camera.out_x:.2f};{self.threat_camera.out_y:.2f}) ; distance [m]: {self.threat_camera.out_range:.2f} ; abs_theta [rad]: {self.threat_camera.out_abs_theta:.2f} ; rel_theta [rad]: {self.threat_camera.in_rel_theta:.2f}"
            # )
        else:
            self.get_logger().info(
                f"[Camera] Threat location estimation failed ; min_distance between points [m]: {min_distance:.2f}"
            )

        # Calcul des coordonnées du point cherché
        X_usv_threat = self.usv_x + self.max_range_usv_threat * math.cos(self.threat_camera.out_abs_theta)
        Y_usv_threat = self.usv_y + self.max_range_usv_threat * math.sin(self.threat_camera.out_abs_theta)
        # Affichage des points
        plt.figure(figsize=(8, 6))
        plt.plot(usv_threat_line[:, 0], usv_threat_line[:, 1], label='Demi-droite entre vous et le point')
        plt.plot(origine_threat_line[:, 0], origine_threat_line[:, 1], label='Demi-droite entre origine et le point')
        plt.plot(self.usv_x, self.usv_y, 'ro', label='Votre position')
        plt.plot(X_usv_threat, Y_usv_threat, 'go', label='Point')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Demi-droites vers le point cherché')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')


def main(args=None):
    rclpy.init()
    env = Environment()
    env.run()


if __name__ == "__main__":
    main()
