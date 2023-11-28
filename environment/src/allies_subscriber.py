from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class AlliesPositionSubscriber(Node):
    def __init__(
        self,
        node
    ):
        self.node = node
        self.init = True
        self.allies_pos = []
        self.allies_subscription = self.node.create_subscription(
            PoseArray,
            "/wamv/ais_sensor/allies_positions",
            self.allies_position_callback,
            10,
        )

    def allies_position_callback(self, msg: PoseArray):
        count = 0
        for pose in msg.poses:
            if self.init: # Si c'est la première fois qu'on reçoit les positions des alliés
                self.allies_pos.append((pose.position.x, pose.position.y)) # Ajout des alliés dans la liste
            else:
                self.allies_pos[count] = (pose.position.x, pose.position.y) # Mise à jour des alliés dans la liste
            count += 1
        if self.init: # On baisse le flag init
            self.init = False
        self.node.allies_position_cb()