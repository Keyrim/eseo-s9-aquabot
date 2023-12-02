import math
from std_msgs.msg import Float64

WAMV_TOPIC_THRUSTERS_POS = '/wamv/thrusters/main/pos'
WAMV_TOPIC_THRUSTERS_THRUST = '/wamv/thrusters/main/thrust'

MAX_POS = math.pi/4 # amplitude max du signal de commande (rad)
MAX_THRUST = 12000  # amplitude max du signal de commande (tr/min)

class ThrustersPublisher:
    def __init__(self, node):
        self.node = node
        self.publisher_thrust = node.create_publisher(Float64, WAMV_TOPIC_THRUSTERS_THRUST, 10)
        self.publisher_pos = node.create_publisher(Float64, WAMV_TOPIC_THRUSTERS_POS, 10)
        self.thrust = 0     # Real thrust sent to the thrusters
        self.pos = 0        # Real pos (rad) sent to the thrusters
        self.u_torque = 0   # Torque control input
        self.u_thrust = 0   # Thrust control input

    def publish(self):
        msg = Float64()
        msg.data = float(self.thrust)
        self.publisher_thrust.publish(msg)
        real_pos = self.pos
        if self.pos > MAX_POS:
            real_pos = MAX_POS
        elif self.pos < -MAX_POS:
            real_pos = -MAX_POS
        msg.data = float(real_pos)
        self.publisher_pos.publish(msg)

    def command(self, thrust, torque):
        self.u_torque = torque
        self.u_thrust = thrust
        self.pos = -math.atan2(self.u_torque, self.u_thrust)
        self.thrust = math.sqrt(self.u_torque**2 + self.u_thrust**2)
        self.publish()

