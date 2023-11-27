import rclpy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np
from rclpy.node import Node
from environment_interfaces.msg import BoatState
import matplotlib.pyplot as plt
from environment_interfaces.msg import LidarCluster


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.buoy_subscription = self.create_subscription(
            LaserScan,
            "/wamv/sensors/lidars/lidar_wamv_sensor/scan",
            self.lidar_callback,
            10,
        )
        self.gps_subscription = self.create_subscription(
            BoatState,
            "/boat/estimator/position",  # TODO Subscribe to Theo filtered GPS topic
            self.gps_callback,
            10,
        )
        self.lidar_cluster_publisher = self.create_publisher(
            LidarCluster,
            "/environment/lidar_cluster",
            10,
        )
        self.usv_x = 0
        self.usv_y = 0
        self.usv_theta = 0
        

    def gps_callback(self, msg: BoatState):
        # self.get_logger().info("BoatState received") # DEBUG
        self.usv_x = msg.x
        self.usv_y = msg.y
        self.usv_theta = msg.theta

    def lidar_callback(self, msg: LaserScan):
        # Initialisation de la figure Matplotlib pour le graphique en temps réel, ici pour être dans le même thread que l'utilisation de Matplotlib
        self.fig, self.ax = plt.subplots()
        self.points_scatter = self.ax.scatter([], [], color="blue")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Visualisation des points Lidar")
        # Définir les limites des axes X et Y
        self.ax.set_xlim(-400, 400)
        self.ax.set_ylim(-400, 400)
        # Extraction des données de distances (ranges) du message LaserScan
        ranges = msg.ranges

        # Conversion des données en coordonnées cartésiennes (x, y) dans le repère absolu tel que (0;0) est le spawn de l'USV
        angles = np.linspace(
            msg.angle_min, msg.angle_max, len(ranges)
        )  # Angles d'acquisition
        lidar_data = []
        for i, distance in enumerate(ranges):
            if not np.isinf(distance) and (
                distance > 0.1
            ):  # Travail dans le range 10cm-130m
                rel_x = self.usv_x + (distance * np.cos(angles[i] + self.usv_theta))
                rel_y = self.usv_y + (distance * np.sin(angles[i] + self.usv_theta))
                lidar_data.append(
                    [rel_x, rel_y]
                )  # Ajout des points dans la liste en coordonnées relatives au bateau

        # Clusterisation avec DBSCAN si des données sont disponibles
        if len(lidar_data) > 0:
            lidar_data = np.array(lidar_data)
            self.points_scatter.set_offsets(lidar_data)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
            eps = 1  # TODO régler les paramètres de DBSCAN
            min_samples = 3  # TODO régler les paramètres de DBSCAN
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

                # Calcul de la distance entre votre position et le point central
                distance = np.linalg.norm(
                    central_point - np.array([self.usv_x, self.usv_y])
                )

                # Calcul de l'angle entre le point central et l'origine du repère
                theta_rad = np.arctan2(central_point[1], central_point[0])

                cluster = LidarCluster()
                cluster.relative_theta = theta_rad - self.usv_theta
                cluster.absolute_theta = theta_rad
                cluster.distance = distance
                cluster.x = central_point[0]
                cluster.y = central_point[1]
                self.lidar_cluster_publisher.publish(cluster)
                self.get_logger().info(
                    f"Cluster {cluster_id:.2f}: pos = ({cluster.x:.2f};{cluster.y:.2f}) ; d = {distance:.2f}, abs_theta_deg : {cluster.absolute_theta:.2f} ; rel_theta_deg : {np.rad2deg(cluster.relative_theta):.2f}"
                )  # DEBUG
            print("\n\n")