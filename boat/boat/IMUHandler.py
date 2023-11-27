import math
from sensor_msgs.msg import Imu

TOPIC_IMU = '/wamv/sensors/imu/imu/data'

class IMUHandler:
    def __init__(self, node):
        self.node = node
        self.theta = 0
        self.subscription = node.create_subscription(
            Imu, in_topic_imu, self.callback, 10)

    def callback(self, msg):
        # Get the orientation of the boat from the IMU
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.theta = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        # Publish the state of the boat
        self.node.publish_state()