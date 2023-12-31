import math
from sensor_msgs.msg import NavSatFix

TOPIC_GPS = '/wamv/sensors/gps/gps/fix'

class GPSHandler:
    def __init__(self, node):
        self.node = node
        self.base_lat = None
        self.base_lon = None
        self.x = 0
        self.y = 0
        self.subscription = node.create_subscription(
            NavSatFix, TOPIC_GPS, self.callback, 10)
        # Speed computation
        self.previous_time_s = 0
        self.previous_time_ns = 0
        self.previous_x = 0
        self.previous_y = 0

    def callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        if self.base_lat is None and self.base_lon is None:
            self.base_lat, self.base_lon = latitude, longitude
        self.x, self.y = self.convert_gps_to_xy(latitude, longitude)
        # Speed computation
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        dt = (sec - self.previous_time_s) + ((nsec - self.previous_time_ns) / 1e9)
        self.vx = (self.x - self.previous_x) / dt
        self.vy = (self.y - self.previous_y) / dt
        self.previous_time_s = sec
        self.previous_time_ns = nsec
        self.previous_x = self.x
        self.previous_y = self.y
        # Publish the state of the boat
        self.node.gps_handler_cb()

    def convert_gps_to_xy(self, latitude, longitude):
        # Haversine distance calculation
        R = 6371000  # Earth radius in meters

        dLat = math.radians(latitude - self.base_lat)
        dLon = math.radians(longitude - self.base_lon)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.sin(dLon / 2) * math.sin(dLon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # Calculate x and y based on the distance and bearing
        y = math.sin(dLon) * math.cos(math.radians(latitude))
        x = math.cos(math.radians(self.base_lat)) * math.sin(math.radians(latitude)) - math.sin(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.cos(dLon)
        bearing = math.atan2(y, x)
        y = distance * math.cos(bearing)
        x = distance * math.sin(bearing)
        return x, y