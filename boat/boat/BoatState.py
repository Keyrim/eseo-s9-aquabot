import math
import environment_interfaces.msg

TOPIC_BOAT_STATE = '/boat/estimator/position'

class BoatStateReceiver:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vx = 0
        self.vy = 0

    def getSpeed(self):
        return math.sqrt(self.vx**2 + self.vy**2)
    
    def getOrientation(self):
        return self.theta
    
    def getPosition(self):
        return (self.x, self.y)
    
    def updateAll(self, x, y, theta, vx, vy):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.vy = vy

    def updateFromMsg(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.vx = msg.vx
        self.vy = msg.vy

class BoatStatePublisher:
    def __init__(self, node):
        self.node = node
        self.publisher_pos = node.create_publisher(BoatState, TOPIC_BOAT_STATE, 10)

    def publish(self):
        msg = BoatState()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.theta = float(self.theta)
        msg.vx = float(self.vx)
        msg.vy = float(self.vy)
        self.publisher_pos.publish(msg)