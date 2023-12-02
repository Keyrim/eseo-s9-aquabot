import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from environment_interfaces.msg import ThreatInfo
import math
from environment_interfaces.msg import BoatState
from environment_interfaces.msg import LidarCluster

base_lat = 48.046300000000  # TODO read it from estimator service
base_lon = -4.976320000000  # TODO read it from estimator service

def convert_gps_to_xy(latitude, longitude):
    # Haversine distance calculation
    R = 6371000  # Earth radius in meters

    dLat = math.radians(latitude - base_lat)
    dLon = math.radians(longitude - base_lon)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(
        math.radians(base_lat)
    ) * math.cos(math.radians(latitude)) * math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    # Calculate x and y based on the distance and bearing
    y = math.sin(dLon) * math.cos(math.radians(latitude))
    x = math.cos(math.radians(base_lat)) * math.sin(
        math.radians(latitude)
    ) - math.sin(math.radians(base_lat)) * math.cos(
        math.radians(latitude)
    ) * math.cos(
        dLon
    )
    bearing = math.atan2(y, x)
    y = distance * math.cos(bearing)
    x = distance * math.sin(bearing)
    return x, y

class Map(Node):
    def __init__(self):
        super().__init__("map")
        self.allies_subscription = self.create_subscription(
            PoseArray,
            "/wamv/ais_sensor/allies_positions",
            self.allies_position_cb,
            10,
        )

        self.buoy_pos_subscription = self.create_subscription(
            Point, "/environment/buoy_pos", self.buoy_pos_cb, 10
        )

        self.threat_pos_subscription = self.create_subscription(
            ThreatInfo, "/environment/threat_info", self.threat_pos_cb, 10
        )

        self.target_pos_subscription = self.create_subscription(
            Point, "/boat/controller/traj", self.target_pos_cb, 10
        )

        self.usv_pos_subscription = self.create_subscription(
            BoatState, "/boat/estimator/position", self.usv_pos_cb, 10
        )

        self.debug_lidar_cluster_subscription = self.create_subscription(
            LidarCluster,
            "/environment/lidar_debug",
            self.lidar_cluster_cb,
            10,
        )

        self.update_map_timer = self.create_timer(
            1,
            self.update_map_timer_cb
        )

        self.init_allies_pos = True
        self.usv_pos = Point()
        self.allies_pos = []
        self.threat_pos = Point()
        self.target_pos = Point()
        self.buoy_pos = Point()
        self.lidar_clusters = []
        self.obstacles = [
            Obstacles(120, -50, 25),
            Obstacles(-152, -6, 50),
            Obstacles(110, 130, 50),
            Obstacles(12, -102, 25),
            Obstacles(92, 170, 25),
            Obstacles(-92, 176, 30),
            Obstacles(-40, 220, 30),
            Obstacles(-44, -95, 30),
            Obstacles(-30, -150, 30),
        ]
        self.fig, self.ax = plt.subplots()

    def update_map_timer_cb(self):
        self.ax.clear()  # Nettoyer les axes à chaque mise à jour
        lidar_clusters = self.lidar_clusters.copy()
        self.lidar_clusters.clear()
        for cluster in lidar_clusters:
                self.ax.plot(cluster[0], cluster[1], marker='o', markersize=3, color='orange', linestyle='') # Lidar clusters in orange

        for x, y in self.allies_pos:
            self.ax.plot(x, y, marker='o', markersize=8, color='green', linestyle='') # Allies in green

        self.ax.plot(self.threat_pos.x, self.threat_pos.y, marker='o', markersize=8, color='red', linestyle='') # Threat in red
        self.ax.plot(self.buoy_pos.x, self.buoy_pos.y, marker='o', markersize=8, color='blue', linestyle='') # Buoy in blue
        self.ax.plot(self.target_pos.x, self.target_pos.y, marker='o', markersize=8, color='grey', linestyle='') # Target in grey
        self.ax.plot(self.usv_pos.x, self.usv_pos.y, marker='o', markersize=8, color='purple', linestyle='') # USV in purple

        for obstacle in self.obstacles:
            circle = plt.Circle(
                (obstacle.x, obstacle.y), obstacle.radius, color="red", fill=False # Obstacles in red
            )
            self.ax.add_patch(circle)

        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)

        plt.xlabel('Axe X')
        plt.ylabel('Axe Y')
        plt.title('Mise à jour de la carte')
        plt.grid(True)
        plt.pause(0.1)  # Mise à jour de l'affichage

    def allies_position_cb(self, msg: PoseArray):
        for count, pose in enumerate(msg.poses):
            if self.init_allies_pos is True: # Si c'est la première fois qu'on reçoit les positions des alliés
                self.allies_pos.append(convert_gps_to_xy(pose.position.x, pose.position.y)) # Ajout des alliés dans la liste
            else:
                self.allies_pos[count] = convert_gps_to_xy(pose.position.x, pose.position.y) # Mise à jour des alliés dans la liste
        if self.init_allies_pos: # On baisse le flag init
            self.init_allies_pos = False
    
    def threat_pos_cb(self, msg: ThreatInfo):
        self.threat_pos.x = msg.x
        self.threat_pos.y = msg.y

    def buoy_pos_cb(self, msg: Point):
        self.buoy_pos.x = msg.x
        self.buoy_pos.y = msg.y

    def target_pos_cb(self, msg: Point):
        self.target_pos.x = msg.x
        self.target_pos.y = msg.y

    def usv_pos_cb(self, msg: BoatState):
        self.usv_pos.x = msg.x
        self.usv_pos.y = msg.y
        
    def lidar_cluster_cb(self, msg: LidarCluster):
        self.lidar_clusters.append((msg.x, msg.y))

class Obstacles:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


def main():
    rclpy.init()
    node = Map()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
