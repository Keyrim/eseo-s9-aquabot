import cv2
import numpy as np
import imutils # Package installé via pip
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from src.threat_position_publisher import ThreatPositionPublisher
from src.buoy_position_publisher import BuoyPositionPublisher
from environment_interfaces.msg import SearchedObjectInfo
from src.shared_types import Source


class FoundElement:
    def __init__(
        self, c_x: int, c_y: int, angle: float, area: float, contour: np.ndarray
    ):
        self.x = c_x
        self.y = c_y
        self.angle = angle
        self.area = area
        self.contour = contour


class ImageSubscriber(Node):
    def __init__(
        self,
        threat_position_publisher: ThreatPositionPublisher,
        buoy_position_publisher: BuoyPositionPublisher,
    ):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(
            Image,
            "/wamv/sensors/cameras/main_camera_sensor/optical/image_raw",
            self.listener_callback,
            10,
        )
        self.subscription  # prevent unused variable warning
        #  Initialiser CvBridge pour convertir les images ROS en images OpenCV
        self.bridge = CvBridge()
        self.threat_position_publisher = threat_position_publisher
        self.buoy_position_publisher = buoy_position_publisher

    def threat_tracker(self, msg: Image):
        fov_deg = 80  # FOV de la caméra
        # Le Y de la menace est toujours supérieur à cette valeur (pour ne pas capter les arbres en hauteur)
        y_threat_threshold = 170
        # TODO à modifier en fonction de la taille de la menace à longue distance
        area_threshold = 1
        px_left_angle_deg = 180 - ((fov_deg / 2) / 2)
        px_right_angle_deg = (fov_deg / 2) / 2

        # Convertir l'image ROS en format OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Définir la plage de couleur rouge de la menace en BGR
        lower_red = np.array([0, 0, 50])
        upper_red = np.array([50, 60, 130])

        # Filtrer l'image pour obtenir uniquement les pixels rouges
        mask = cv2.inRange(image, lower_red, upper_red)
        # cv2.imshow("Masque Rouge", mask) # DEBUG

        # Trouver les contours des objets rouges dans le range
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        # Dessiner les contours sur l'image originale
        found_elements = []
        for contour in contours:
            area = cv2.contourArea(contour)
            # print("Surface = ", area) # DEBUG
            if area > area_threshold:
                # compute the center of the contour
                M = cv2.moments(contour)
                cX, cY = 0, 0
                if M["m00"] != 0 and M["m01"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                threat_angle = (
                    px_left_angle_deg
                    + ((px_right_angle_deg - px_left_angle_deg) / msg.width) * cX
                )
                found_elements.append(FoundElement(cX, cY, threat_angle, area, contour))

        # Trouver la menace parmi les éléments trouvés en se basant sur la surface et la position en Y
        threat_element = FoundElement(0, 0, 0, 0, None)
        for element in found_elements:
            threat_element = (
                element
                if (
                    element.area > threat_element.area
                    and element.y > y_threat_threshold
                )
                else threat_element
            )
            cv2.drawContours(image, [element.contour], -1, (0, 255, 0), 2)
            cv2.circle(image, (element.x, element.y), 3, (255, 255, 255), -1)
            # cv2.putText(image, f"center : ({element.x};{element.y};{element.angle}°)", (element.x - 20, element.y - 20), 
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) # DEBUG

        if threat_element.area > 0:
            cv2.circle(image, (threat_element.x, threat_element.y), 3, (0, 0, 255), -1)
            cv2.putText(
                image,
                f"area : {threat_element.area} (X;Y) : ({threat_element.x};{threat_element.y}) theta_deg {threat_element.angle:.2f})",
                (threat_element.x - 20, threat_element.y - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (0, 0, 255),
                2,
            )

        threat_info = SearchedObjectInfo()
        threat_info.source = Source.CAMERA
        threat_info.is_found = threat_element.area > 0
        threat_info.angle = math.radians(threat_element.angle)
        threat_info.distance = 0.0
        self.threat_position_publisher.publish_threat_position(threat_info)
        # Afficher l'image
        cv2.imshow("Threat tracker CAMERA", image)
        cv2.waitKey(1)

    def buoy_tracker(self, msg: Image):
        buoy_info = SearchedObjectInfo()
        # TODO set buoy_info fields
        self.buoy_position_publisher.publish_buoy_position(buoy_info)

    def listener_callback(self, msg):
        self.threat_tracker(msg)
        self.buoy_tracker(msg)
