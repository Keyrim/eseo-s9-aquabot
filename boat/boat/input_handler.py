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
        self.dx = 0
        self.dy = 0

    def calculate_error(self):
        # Distance error between trajectory_receiver and boat
        self.dx = self.trajectory_receiver.x - self.boat_state_receiver.x
        self.dy = self.trajectory_receiver.y - self.boat_state_receiver.y
        self.e_d = math.sqrt(self.dx**2 + self.dy**2)
        # Angle error between trajectory_receiver and boat
        self.e_theta = math.atan2(self.dy, self.dx) - self.boat_state_receiver.theta
        # Keep angle error between -pi and pi
        if self.e_theta > math.pi:
            self.e_theta -= 2 * math.pi
        elif self.e_theta < -math.pi:
            self.e_theta += 2 * math.pi
        self.node.input_handler_cb()