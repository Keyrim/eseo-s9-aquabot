from geometry_msgs.msg import Point


TOPIC_BUOY = "/environment/buoy_pos"

class BuoyReceiver:
    def __init__(self, node):
        self.node = node
        self.point = Point()
        self.subscription = node.create_subscription(
            Point, TOPIC_BUOY, self.callback, 10)

    def callback(self, msg):
        self.order = msg.order
        self.node.buoy_receiver_cb()

    def get_buoy_pos(self):
        return (self.point.x, self.point.y)