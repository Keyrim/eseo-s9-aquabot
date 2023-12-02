import math
from environment_interfaces.msg import BoatState

TOPIC_BOAT_STATE = '/boat/estimator/position'

class BoatStateReceiver:
    def __init__(self, node):
        self.node = node
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vx = 0
        self.vy = 0
        self.subscription = node.create_subscription(
            BoatState, TOPIC_BOAT_STATE, self.callback, 10)

    def get_speed(self):
        return math.sqrt(self.vx**2 + self.vy**2)

    def callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.vx = msg.vx
        self.vy = msg.vy
        self.node.boat_state_receiver_cb()

    def get_pos(self):
        return (self.x, self.y)

class BoatStatePublisher:
    def __init__(self, node):
        self.node = node
        self.publisher_pos = node.create_publisher(BoatState, TOPIC_BOAT_STATE, 10)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vx = 0
        self.vy = 0
        self.base_lat = None
        self.base_lon = None

    def set_base_lat_lon(self, base_lat, base_lon):
        self.base_lat = base_lat
        self.base_lon = base_lon

    def publish(self):
        msg = BoatState()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.theta = float(self.theta)
        msg.vx = float(self.vx)
        msg.vy = float(self.vy)
        msg.base_lat = float(self.base_lat)
        msg.base_lon = float(self.base_lon)
        self.publisher_pos.publish(msg)

    def update_pos_and_speed(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy

    def update_orientation(self, theta):
        self.theta = theta