from environment_interfaces.msg import BoatTrajectory

TOPIC_BOAT_TARGET = '/boat/controller/traj'

class TrajectoryReceiver:
    def __init__(self, node):
        self.x = 0
        self.y = 0
        self.full_speed = False
        self.node = node
        self.subscription = node.create_subscription(
            BoatTrajectory, TOPIC_BOAT_TARGET, self.callback, 10)

    def callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.full_speed = msg.full_speed
        self.node.trajectory_receiver_cb()


class TrajectoryPublisher:
    def __init__(self, node):
        self.node = node
        self.publisher_pos = node.create_publisher(BoatTrajectory, TOPIC_BOAT_TARGET, 10)

    def publish(self, x, y, full_speed):
        msg = BoatTrajectory()
        msg.x = float(x)
        msg.y = float(y)
        msg.full_speed = full_speed
        self.publisher_pos.publish(msg)

