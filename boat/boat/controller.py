import rclpy
import math
from rclpy.node import Node
from boat.trajectory import TrajectoryReceiver
from boat.boat_state import BoatStateReceiver
from boat.thrusters import ThrustersPublisher

# PID gains
KP_THRUST = 200
KI_THRUST = 0
KD_THRUST = 0
KP_TORQUE = 10
KI_TORQUE = 0
KD_TORQUE = 0

# Configuration
MIN_DISTANCE = 3 # Pas d'asservissement si la distance entre la cible et le bateau est inférieure à MIN_DISTANCE

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


class PidController:
    def __init__(self, kp, ki, kd, max_u=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e = 0          # Error
        self.e_sum = 0      # Sum of errors
        self.e_prev = 0     # Previous error
        self.u = 0          # Control signal
        self.max_u = max_u  # Maximum control signal

    def compute(self, e, dt):
        self.e = e
        self.e_sum += e*dt
        self.u = self.kp*e #+ self.ki*self.e_sum + self.kd*(e - self.e_prev)/dt
        if self.max_u != 0:
            self.u = min(self.u, self.max_u)
        self.e_prev = e
        return self.u


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.input_handler = InputHandler(self)
        self.thrusters = ThrustersPublisher(self)
        self.pid_d = PidController(KP_THRUST, KI_THRUST, KD_THRUST, 20000)
        self.pid_theta = PidController(KP_TORQUE, KI_TORQUE, KD_TORQUE)

    def input_handler_cb(self):
        if(self.input_handler.e_d > MIN_DISTANCE):
            # Two configuration, either we are "kinda ok" regarding the angle error*
            if abs(self.input_handler.e_theta) < math.pi/2:
                # In this case we set a minimum speed because we gotta move
                real_e_d = min(self.input_handler.e_d, 200)
            # Or we are not, and we limit the thrust because we gotta turn first
            else:
                real_e_d = max(self.input_handler.e_d, 200)
            u_d = self.pid_d.compute(real_e_d, 0.1)
            u_torque = self.pid_theta.compute(self.input_handler.e_theta, 0.1)

            # Log error and control signal adn real e d with 2 decimals
            self.get_logger().info('e_d: %f, e_theta: %f, u_d: %f, u_torque: %f, real_e_d: %f' % (self.input_handler.e_d, self.input_handler.e_theta, u_d, u_torque, real_e_d))
            self.control(u_d, u_torque)

    def trajectory_receiver_cb(self):
        self.input_handler.calculate_error()

    def boat_state_receiver_cb(self):
        self.input_handler.calculate_error()

    def control(self, u_d, u_theta):
        thrust = u_d*math.cos(u_theta)
        angle = u_theta
        self.thrusters.command(thrust, angle)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
