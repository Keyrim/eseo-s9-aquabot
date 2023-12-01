import math
from boat.trajectory import TrajectoryReceiver
from boat.boat_state import BoatStateReceiver

class InputHandler:
    def __init__(self, node):
        self.node = node
        self.trajectory_receiver = TrajectoryReceiver(node)
        self.boat_state_receiver = BoatStateReceiver(node)
        # Error between trajectory_receiver and boat
        self.e_d = 0
        self.e_theta = 0

    def calculate_error(self):
        # Distance error between trajectory_receiver and boat
        dx = self.trajectory_receiver.x - self.boat_state_receiver.x
        dy = self.trajectory_receiver.y - self.boat_state_receiver.y
        self.e_d = math.sqrt(dx**2 + dy**2)
        # Angle error between trajectory_receiver and boat
        self.e_theta = math.atan2(dy, dx) - self.boat_state_receiver.theta
        # Keep the angle error between -pi and pi
        # if self.e_theta > math.pi:
        #     self.e_theta -= 2*math.pi
        # elif self.e_theta < -math.pi:
        #     self.e_theta += 2*math.pi

        # Log both errors
        self.node.get_logger().info('e_d: %f, e_theta: %f' % (self.e_d, self.e_theta))
        self.node.input_handler_cb()