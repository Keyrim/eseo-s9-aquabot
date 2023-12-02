import cv2
import numpy as np
import imutils  # Package installé via pip
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from environment_interfaces.msg import ThreatInfo
# from environment_interfaces.msg import BuoyInfo

class CameraFoundElement:
    def __init__(
        self, c_x: int, c_y: int, angle: float, area: float, contour: np.ndarray
    ):
        self.x = c_x
        self.y = c_y
        self.angle = angle
        self.area = area
        self.contour = contour


class ImageSubscriber():
    def __init__(
        self,
        node: Node,
    ):
        self.node = node
        self.image_subscription = self.node.create_subscription(
            Image,
            "/wamv/sensors/cameras/main_camera_sensor/optical/image_raw",
            self.listener_callback,
            10,
        )
        # self.buoy_position_publisher = self.node.create_publisher(
        #     BuoyInfo, "/environment/buoy_position", 10
        # )
        #  Initialiser CvBridge pour convertir les images ROS en images OpenCV
        self.bridge = CvBridge()
        self.fov_deg = 80  # FOV de la caméra
        self.px_left_angle_deg = self.fov_deg / 2
        self.px_right_angle_deg = -self.fov_deg / 2
        self.threat_lost_published_flag = (
            False  # 1 seule publication quand la menace est perdue de vue
        )
        self.buoy_lost_published_flag = (
            False  # 1 seule publication quand la bouée est perdue de vue
        )
        self.is_threat_found = False
        self.threat_theta = math.inf
        self.threat_image: Image = None
        self.rx_counter = 0

    def threat_tracker(self, msg: Image):
        # Le Y de la menace est toujours supérieur à cette valeur (pour ne pas capter les arbres en hauteur)
        y_threat_threshold = 170
        # TODO à modifier en fonction de la taille de la menace à longue distance
        area_threshold = 1

        # Convertir l'image ROS en format OpenCV BGR
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Définir la plage de couleur rouge de la menace en BGR
        lower_red = np.array([0, 0, 50])
        upper_red = np.array([50, 60, 130])

        # Filtrer l'image pour obtenir uniquement les pixels rouges
        mask = cv2.inRange(image, lower_red, upper_red)
        # cv2.imshow("Threat mask", mask) # DEBUG

        # Trouver les contours des objets rouges dans le range
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        # Dessiner les contours sur l'image originale
        found_elements = []
        for contour in contours:
            area = cv2.contourArea(contour)
            # self.get_logger().info("Surface = ", area) # DEBUG
            if area > area_threshold:
                # compute the center of the contour
                M = cv2.moments(contour)
                cX, cY = 0, 0
                if M["m00"] != 0 and M["m01"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                deg_per_px = self.fov_deg / msg.width
                threat_angle = self.px_right_angle_deg + (
                    deg_per_px * (msg.width - cX)
                )  # Max angle à gauche, min angle à droite (trigo)

                found_elements.append(
                    CameraFoundElement(cX, cY, threat_angle, area, contour)
                )

        # Trouver la menace parmi les éléments trouvés en se basant sur la surface et la position en Y
        threat_element = CameraFoundElement(0, 0, 0, 0, None)
        for element in found_elements:
            threat_element = (
                element
                if (
                    element.area > threat_element.area
                    and element.y > y_threat_threshold
                )
                else threat_element
            )
            # cv2.drawContours(image, [element.contour], -1, (0, 255, 0), 2)
            # cv2.circle(image, (element.x, element.y), 3, (255, 255, 255), -1)
            # cv2.putText(image, f"center : ({element.x};{element.y};{element.angle}°)", (element.x - 20, element.y - 20),
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) # DEBUG

        # if threat_element.area > 0:
            # cv2.circle(image, (threat_element.x, threat_element.y), 3, (0, 0, 255), -1)
            # cv2.putText(
                # image,
                # f"area : {threat_element.area} (X;Y) : ({threat_element.x};{threat_element.y}) theta_deg {threat_element.angle:.2f})", # DEBUG
                # f"theta_threat [-40;40 deg] {threat_element.angle:.2f}",
                # (threat_element.x - 70, threat_element.y - 20),
                # cv2.FONT_HERSHEY_SIMPLEX,
                # 0.3,
                # (0, 0, 255),
                # 2,
            # )

        is_threat_found = threat_element.area > 0  # True si la menace est trouvée
        # Publier la menace si visible ou si on vient de la perdue de vue
        if is_threat_found is True or self.threat_lost_published_flag is False:
            # Si la menace est perdue de vue, publier une menace avec is_found = False et lever le flag
            if is_threat_found is False:
                self.threat_lost_published_flag = True  # On ne rentrera dans le if N+1 que si on repère à nouveau la menace
            else:
                self.threat_lost_published_flag = (
                    False  # Menace repérée, on baisse le flag
                )
            self.is_threat_found = is_threat_found
            self.threat_theta = math.radians(threat_element.angle)
            # self.threat_image = image
            self.node.image_listener_threat_cb()

        # Afficher l'image
        # cv2.imshow("Threat tracker CAMERA", image)
        # cv2.waitKey(1)

    # def buoy_tracker(self, msg: Image):
    #     y_buoy_threshold = 170
    #     area_threshold = 1
    #     # Convertir l'image ROS en format OpenCV BGR
    #     image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    #     # Définir la plage de couleur jaune de la bouée en BGR
    #     lower_buoy = np.array([10, 100, 100])
    #     upper_buoy = np.array([34, 150, 160])

    #     # Filtrer l'image pour obtenir uniquement les pixels dans le range
    #     mask = cv2.inRange(image, lower_buoy, upper_buoy)
    #     # cv2.imshow("Buoy mask", mask) # DEBUG

    #     # Trouver les contours des objets du range
    #     contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     contours = imutils.grab_contours(contours)

    #     # Dessiner les contours sur l'image originale
    #     found_elements = []
    #     for contour in contours:
    #         area = cv2.contourArea(contour)
    #         # self.get_logger().info("Surface = ", area) # DEBUG
    #         if area > area_threshold:
    #             # compute the center of the contour
    #             M = cv2.moments(contour)
    #             cX, cY = 0, 0
    #             if M["m00"] != 0 and M["m01"] != 0:
    #                 cX = int(M["m10"] / M["m00"])
    #                 cY = int(M["m01"] / M["m00"])
    #             deg_per_px = self.fov_deg / msg.width
    #             buoy_angle = self.px_right_angle_deg + (
    #                 deg_per_px * (msg.width - cX)
    #             )  # Max angle à gauche, min angle à droite (sens trigo)
    #             found_elements.append(
    #                 CameraFoundElement(cX, cY, buoy_angle, area, contour)
    #             )

    #     # Trouver la bouée parmi les éléments trouvés en se basant sur la surface et la position en Y
    #     buoy_element = CameraFoundElement(0, 0, 0, 0, None)
    #     for element in found_elements:
    #         buoy_element = (
    #             element
    #             if (element.area > buoy_element.area and element.y > y_buoy_threshold)
    #             else buoy_element
    #         )
    #         cv2.drawContours(image, [element.contour], -1, (255, 0, 0), 2)
    #         cv2.circle(image, (element.x, element.y), 3, (255, 255, 0), -1)
    #         # cv2.putText(image, f"center : ({element.x};{element.y};{element.angle}°)", (element.x - 20, element.y - 20),
    #         #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) # DEBUG

    #     is_buoy_found = buoy_element.area > 0  # True si la bouée est trouvée
    #     if is_buoy_found is True:
    #         cv2.circle(image, (buoy_element.x, buoy_element.y), 3, (255, 0, 0), -1)
    #         cv2.putText(
    #             image,
    #             # f"area : {buoy_element.area} (X;Y) : ({buoy_element.x};{buoy_element.y}) theta_deg {buoy_element.angle:.2f})", #DEBUG
    #             f"theta_buoy [-40;40 deg] {buoy_element.angle:.2f}",
    #             (buoy_element.x - 70, buoy_element.y - 20),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             0.3,
    #             (0, 0, 255),
    #             2,
    #         )

    #     # Publier la bouée si visible ou si on vient de la perdue de vue
    #     if is_buoy_found is True or self.buoy_lost_published_flag is False:
    #         # Si la bouée est perdue de vue, publier une bouée avec is_found = False et lever le flag
    #         if is_buoy_found is False:
    #             self.buoy_lost_published_flag = True  # On ne rentrera dans le if N+1 que si on repère à nouveau la bouée
    #         else:
    #             self.buoy_lost_published_flag = (
    #                 False  # bouée repérée, on baisse le flag
    #             )
    #         # Publier les infos de la bouée
    #         buoy_info = BuoyInfo()
    #         buoy_info.source = Source.CAMERA
    #         buoy_info.is_found = is_buoy_found
    #         buoy_info.angle = math.radians(buoy_element.angle)
    #         buoy_info.distance = 0.0
    #         self.buoy_position_publisher.publish(buoy_info)

    #     # Afficher l'image
    #     cv2.imshow("Buoy tracker CAMERA", image)
    #     cv2.waitKey(1)

    def listener_callback(self, msg):
        # 2 trackers différents pour la menace et la bouée, appelés à tour de rôle pour limiter la charge CPU et le temps de traitement
        
        if self.rx_counter == 0:
            self.threat_tracker(msg)
        elif self.rx_counter == 2:
            pass
        #     self.buoy_tracker(msg)
        self.rx_counter = (self.rx_counter + 1) % 2
