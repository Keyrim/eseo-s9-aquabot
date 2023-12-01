import rclpy
import math
from rclpy.node import Node
from boat.input_handler import InputHandler
from boat.thrusters import ThrustersPublisher
from boat.pid import PidController
from boat.stop_start import StartStopReceiver

# PID gains
KP_THRUST = 200
KI_THRUST = 0
KD_THRUST = 0
KP_TORQUE = 10
KI_TORQUE = 0
KD_TORQUE = 0

# Configuration
MIN_DISTANCE = 3 # Pas d'asservissement si la distance entre la cible et le bateau est inférieure à MIN_DISTANCE

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.input_handler = InputHandler(self)
        self.thrusters = ThrustersPublisher(self)
        self.pid_d = PidController(KP_THRUST, KI_THRUST, KD_THRUST, 20000)
        self.pid_theta = PidController(KP_TORQUE, KI_TORQUE, KD_TORQUE)
        self.start_stop_receiver = StartStopReceiver(self)
        self.timer = self.create_timer(0.1, self.main)

        # Logic variables
        self.working = False

    def input_handler_cb(self):
        # There is nothing to do, we process the date in the main loop on a timer
        # Log error and control signal and real e d with 2 decimals
        pass

    def compute_command(self):
         # Two configuration, either we are "kinda ok" regarding the angle error*
        if abs(self.input_handler.e_theta) < math.pi/2:
            # In this case we set a minimum spee=)d because we gotta move
            real_e_d = min(self.input_handler.e_d, 200)
        # Or we are not, and we limit the thrust because we gotta turn first
        else:
            real_e_d = max(self.input_handler.e_d, 200)
        self.pid_d.compute(real_e_d, 0.1)
        self.pid_theta.compute(self.input_handler.e_theta, 0.1)

    def main(self):
        # Log error and control signal and real e d with 2 decimals
        self.get_logger().info('e_d: %.2f, e_theta: %.2f, u_d: %.2f, u_theta: %.2f' % (
            self.input_handler.e_d, self.input_handler.e_theta,
            self.pid_d.u, self.pid_theta.u))
        # If we are not working, we do nothing
        if not self.working:
            return
        # If we are too close to the target, we do nothing
        if self.input_handler.e_d < MIN_DISTANCE:
            return
        # Compute the control signal
        self.compute_command()
        # Send the control signal to the thrusters
        self.control(self.pid_d.u, self.pid_theta.u)

    def start_stop_receiver(self):
        self.working = self.start_stop_receiver.order == 1

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
