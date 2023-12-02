import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from environment_interfaces.msg import ThreatInfo
from environment_interfaces.msg import LidarCluster
from boat.boat_state import BoatStateReceiver
from src.image_subscriber import ImageSubscriber
from src.buoy_pinger_subscriber import BuoyPingerSubscriber
from src.allies_subscriber import AlliesPositionSubscriber
from sklearn.cluster import DBSCAN

class Obstacles:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

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
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            "/wamv/sensors/lidars/lidar_wamv_sensor/scan",
            self.lidar_callback,
            10,
        )
        self.buoy_pos_publisher = self.create_publisher(
            Point, "/environment/buoy_pos",
            10
        )
        self.compute_threat_position_timer = self.create_timer(
            0.1,
            self.compute_threat_position_timer_cb
        )
        self.threat_info_publisher = self.create_publisher(
            ThreatInfo, "/environment/threat_info",
            10
        )
        self.debug_lidar_publisher = self.create_publisher(
            LidarCluster,
            "/environment/lidar_debug",
            10
        )
        # Subscribers
        self.threat_image_subscriber = ImageSubscriber(self)
        self.boat_state_receiver = BoatStateReceiver(self)
        self.buoy_pinger_subscriber = BuoyPingerSubscriber(self)
        self.allies_position_subscriber = AlliesPositionSubscriber(self)
        
        # Class attributes
        # CONSTANTS
        self.base_lat = 48.046300000000
        self.base_lon = -4.976320000000
        self.keepout_radius = 15
        self.obstacles = [
            Obstacles(120, -50, 25 + self.keepout_radius),
            Obstacles(-152, -6, 50 + self.keepout_radius),
            Obstacles(110, 130, 50 + self.keepout_radius),
            Obstacles(12, -102, 25 + self.keepout_radius),
            Obstacles(92, 170, 25 + self.keepout_radius),
            Obstacles(-92, 176, 30 + self.keepout_radius),
            Obstacles(-40, 220, 30 + self.keepout_radius),
            Obstacles(-44, -95, 30 + self.keepout_radius),
            Obstacles(-30, -150, 30 + self.keepout_radius),
        ]
        # USV info
        self.usv_x = 0.0
        self.usv_y = 0.0
        self.usv_theta = 0.0
        # Camera management : scan threat
        self.threat_camera = CameraObject()
        # Allies positions
        self.allies_pos_xy = []
        # Buoy position
        self.buoy_pos_xy = Point()
        # Threat info
        self.threat_info = ThreatInfo()
        # Lidar clusters
        self.lidar_objects = []
        

    def are_near(self, x1, y1, x2, y2, threshold: float):
        distance = math.hypot(x2 - x1, y2 - y1)
        return distance < threshold
    
    def compute_threat_position_timer_cb(self):
        lidar_objects = self.lidar_objects.copy()
        self.lidar_objects.clear()
        threshold = 5  # Seuil de distance
        to_remove = set()

        # Supprimer les groupes de points proches les uns des autres à un seuil de distance
        for i, cluster1 in enumerate(lidar_objects):
            for j, cluster2 in enumerate(lidar_objects):
                if i != j and self.are_near(cluster1.x, cluster1.y, cluster2.x, cluster2.y, threshold):
                    # Si deux points sont très proches l'un de l'autre, marquez l'index du point à supprimer
                    to_remove.add(j)

        # Supprimer les éléments identifiés à supprimer
        lidar_objects = [cluster for k, cluster in enumerate(lidar_objects) if k not in to_remove]
        to_remove.clear() # Clear pour réutiliser
        if len(lidar_objects) == 0:
            self.threat_info.is_found = False
            return
        ####################################################################################################
        # Supprimer les groupes de points proches de la bouée, des alliés et des obstacles
        for l, cluster in enumerate(lidar_objects):
            # Marqueur pour savoir si on doit supprimer ce groupe
            should_remove = False

            # Supprimer le groupe si proche de la bouée
            if self.are_near(cluster.x, cluster.y, self.buoy_pos_xy.x, self.buoy_pos_xy.y, threshold):
                should_remove = True

            # Si le groupe est proche de la bouée, passe au groupe suivant
            if should_remove:
                to_remove.add(l)
                continue

            # Supprimer le groupe s'il est proche d'un allié
            for ally in self.allies_pos_xy:
                if self.are_near(cluster.x, cluster.y, ally[0], ally[1], threshold):
                    should_remove = True
                    break  # Sort de la boucle si un allié est proche

            if should_remove:
                to_remove.add(l)
                continue

            # Supprimer le groupe s'il est proche d'un obstacle (à définir)
            for obstacle in self.obstacles:
                if self.are_near(cluster.x, cluster.y, obstacle.x, obstacle.y, obstacle.radius):
                    should_remove = True
                    break  # Sort de la boucle si un obstacle est proche
                # else:
                    # self.get_logger().info(f"[threat_pos]: cluster ({cluster.x:.2f};{cluster.y:.2f}) is not near obstacle ({obstacle.x:.2f};{obstacle.y:.2f})")

            if should_remove:
                to_remove.add(l)
                continue
        
        # Supprimer les éléments identifiés à supprimer
        # self.get_logger().info(f"[threat_pos]: removed {len(to_remove)} clusters out of {len(lidar_objects)}, {len(lidar_objects) - len(to_remove)} clusters remaining") # DEBUG
        lidar_objects = [cluster for i, cluster in enumerate(lidar_objects) if i not in to_remove]
        to_remove.clear()

        ####################################################################################################
        # for cluster in lidar_objects: # DEBUG
        #     self.debug_lidar_publisher.publish(cluster) # DEBUG
        
        # S'il n'y a plus d'objets, on n'a pas trouvé la menace
        if len(lidar_objects) == 0:
            self.threat_info.is_found = False
            return

        if self.threat_camera.in_is_found is True:
            # Si la caméra voit la menace, on regarde l'angle de la menace vue par la caméra
            for i, cluster in enumerate(lidar_objects):
                # self.get_logger().info(f"cluster : ({cluster.x:.2f};{cluster.y:.2f}) cluster_theta: {math.degrees(cluster.relative_theta):.2f} ; camera_theta {math.degrees(self.threat_camera.in_rel_theta):.2f} + usv_theta {math.degrees(self.usv_theta):.2f} = : {math.degrees(self.threat_camera.in_rel_theta + self.usv_theta):.2f}")
                if abs(cluster.relative_theta - (self.threat_camera.in_rel_theta + self.usv_theta)) < 0.31: # 0.31 rad ~ 5 % d'erreur
                    self.threat_info.is_found = True
                    (self.threat_info.lon, self.threat_info.lat) = self.convert_gps_to_xy(cluster.x, cluster.y)
                    self.threat_info.x = cluster.x
                    self.threat_info.y = cluster.y
                    self.threat_info.relative_angle = cluster.relative_theta
                    self.threat_info.range = math.hypot(self.usv_x - cluster.x, self.usv_y - cluster.y)
                    self.threat_info_publisher.publish(self.threat_info)
                    self.get_logger().info("Threat found with camera theta arbitration")
        # else:
        #     self.get_logger().info("Threat found (only 1 item left)")
        #     # Si un seul objet reste, on considère que c'est la menace
        #     self.threat_info.is_found = True
        #     (self.threat_info.lon, self.threat_info.lat) = self.convert_gps_to_xy(lidar_objects[0].x, lidar_objects[0].y)
        #     self.threat_info.x = lidar_objects[0].x
        #     self.threat_info.y = lidar_objects[0].y
        #     self.threat_info.relative_angle = lidar_objects[0].relative_theta
        #     self.threat_info.range = math.hypot(self.usv_x - lidar_objects[0].x, self.usv_y - lidar_objects[0].y)
        #     self.threat_info_publisher.publish(self.threat_info)

    def lidar_callback(self, msg: LaserScan):
        ranges = msg.ranges

        # Conversion des données en coordonnées cartésiennes (x, y) dans le repère absolu tel que (0;0) est le spawn de l'USV
        angles = np.linspace(
            msg.angle_min, msg.angle_max, len(ranges)
        )  # Angles d'acquisition
        lidar_data = []
        for i, distance in enumerate(ranges):
            if not np.isinf(distance):
                rel_x = self.usv_x + (distance * np.cos(angles[i] + self.usv_theta))
                rel_y = self.usv_y + (distance * np.sin(angles[i] + self.usv_theta))
                lidar_data.append(
                    [rel_x, rel_y]
                )  # Ajout des points dans la liste en coordonnées relatives au bateau

        # Clusterisation avec DBSCAN si des données sont disponibles
        if len(lidar_data) > 0:
            lidar_data = np.array(lidar_data)
            eps = 0.5  # TODO régler les paramètres de DBSCAN
            min_samples = 5  # TODO régler les paramètres de DBSCAN
            dbscan = DBSCAN(eps=eps, min_samples=min_samples)
            clusters = dbscan.fit_predict(lidar_data)

            # Calcul de la distance et de l'angle pour chaque cluster
            for cluster_id in np.unique(clusters):
                cluster_points = [
                    lidar_data[i]
                    for i, cluster in enumerate(clusters)
                    if cluster == cluster_id
                ]
                central_point = np.mean(
                    cluster_points, axis=0
                )  # Point central du cluster

                # Calcul de la distance entre l'USV et le point central
                distance = np.linalg.norm(
                    central_point - np.array([self.usv_x, self.usv_y])
                )

                # Calcul de l'angle entre le point l'USV et le point central
                theta_rad = np.arctan2(central_point[1] - self.usv_y, central_point[0] - self.usv_x)

                cluster = LidarCluster()
                cluster.relative_theta = theta_rad
                cluster.range = distance
                cluster.x = central_point[0]
                cluster.y = central_point[1]
                # self.get_logger().info(
                #     f"[Lidar cluster] {cluster_id:.2f}: pos = ({cluster.x:.2f};{cluster.y:.2f}) ; d = {distance:.2f}, abs_theta_deg : {cluster.absolute_theta:.2f} ; rel_theta_deg : {np.rad2deg(cluster.relative_theta):.2f}"
                # )  # DEBUG
                self.lidar_objects.append(cluster)

    def image_listener_threat_cb(self):
        self.threat_camera.in_is_found = self.threat_image_subscriber.is_threat_found
        self.threat_camera.in_rel_theta = self.threat_image_subscriber.threat_theta

    def boat_state_receiver_cb(self):  # called by BoatStateReceiver callback
        # self.get_logger().info("BoatState received") # DEBUG
        self.usv_x = self.boat_state_receiver.x
        self.usv_y = self.boat_state_receiver.y
        self.usv_theta = self.boat_state_receiver.theta

    def buoy_pinger_cb(self):
        buoy_range = self.buoy_pinger_subscriber.buoy_range
        buoy_theta = self.buoy_pinger_subscriber.buoy_theta
        self.buoy_pos_xy.x = self.usv_x + (
            buoy_range * math.cos(buoy_theta + self.usv_theta)
        )  # Consider USV orientation
        self.buoy_pos_xy.y = self.usv_y + (
            buoy_range * math.sin(buoy_theta + self.usv_theta)
        )  # Consider USV orientation
        # self.get_logger().info(
        #     f"[buoy] pos ({self.buoy_pos_xy.x:.2f};{self.buoy_pos_xy.y:.2f}) ; pinger {buoy_theta:.2f} ; usv {self.usv_theta:.2f}"
        # )
        self.buoy_pos_publisher.publish(self.buoy_pos_xy)

    def allies_position_cb(self):
        init_flag = len(self.allies_pos_xy) == 0
        for count, ally_pos in enumerate(self.allies_position_subscriber.allies_pos):
            if init_flag is True:
                self.allies_pos_xy.append(
                    self.convert_gps_to_xy(ally_pos[0], ally_pos[1])
                )
            else:
                self.allies_pos_xy[count] = self.convert_gps_to_xy(
                    ally_pos[0], ally_pos[1]
                )
                # print(f"Ally[{count} : ({self.allies_pos_xy[count][0]:.2f};{self.allies_pos_xy[count][1]:.2f})]") # DEBUG

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

    def convert_xy_to_gps(self, x, y):
        R = 6371000  # Rayon de la Terre en mètres
        base_lat_radians = math.radians(self.base_lat)
        base_lon_radians = math.radians(self.base_lon)

        distance = math.sqrt(x**2 + y**2)
        bearing = math.atan2(y, x)

        latitude = math.asin(
            math.sin(base_lat_radians) * math.cos(distance / R) +
            math.cos(base_lat_radians) * math.sin(distance / R) * math.cos(bearing)
        )
        longitude = base_lon_radians + math.atan2(
            math.sin(bearing) * math.sin(distance / R) * math.cos(base_lat_radians),
            math.cos(distance / R) - math.sin(base_lat_radians) * math.sin(latitude)
        )

        latitude = math.degrees(latitude)
        longitude = math.degrees(longitude)

        return latitude, longitude

def main(args=None):
    print("Environment started")
    rclpy.init(args=args)
    node = Environment()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
