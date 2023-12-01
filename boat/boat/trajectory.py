from environment_interfaces.msg import BoatTrajectory

TOPIC_BOAT_TARGET = '/boat/controller/traj'

class TrajectoryReceiver:
    def __init__(self, node):
        self.x = -30
        self.y = -150
        self.node = node
        self.subscription = node.create_subscription(
            BoatTrajectory, TOPIC_BOAT_TARGET, self.callback, 10)

    def callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.node.trajectory_receiver_cb()

