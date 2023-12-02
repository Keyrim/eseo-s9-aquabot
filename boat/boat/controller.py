import rclpy
import math
from rclpy.node import Node
from boat.input_handler import InputHandler
from boat.thrusters import ThrustersPublisher
from boat.pid import PidController
from boat.controller_mode import ControllerModeReceiver, ControllerMode
from boat.vector_2d_plotter import Vector2DPlotter

# PID gains
KP_THRUST = 200
KI_THRUST = 0
KD_THRUST = 0
MAX_THRUST = 10000

KP_TORQUE = 130
KI_TORQUE = 0
KD_TORQUE = 0

# Configuration
MIN_DISTANCE = 2 # Pas d'asservissement si la distance entre la cible et le bateau est inférieure à MIN_DISTANCE
THROTLE_FOR_REVERSE = 2000
PLOT_COMMAND_VECTOR = False # Affiche le vecteur de commande en live (sans bloquer l'exécution)

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.input_handler = InputHandler(self)
        self.thrusters = ThrustersPublisher(self)
        self.pid_d = PidController(KP_THRUST, KI_THRUST, KD_THRUST, MAX_THRUST)
        self.pid_torque = PidController(KP_TORQUE, KI_TORQUE, KD_TORQUE)
        self.controller_mode_publisher = ControllerModeReceiver(self)
        self.timer = self.create_timer(0.001, self.main)

        # Plotting
        if PLOT_COMMAND_VECTOR:
            self.plot_command_timer = self.create_timer(0.01, self.plot_command_timer_cb)
            self.command_ploter = Vector2DPlotter(
                num_vectors=3,
                plot_size=(1000, 1000),
                vector_labels=['Command', 'Error', 'Boat direction']
            )
            self.command_ploter.show()

        # Logic variables
        self.working = False

    def input_handler_cb(self):
        # There is nothing to do, we process the date in the main loop on a timer
        # Log error and control signal and real e d with 2 decimals
        if PLOT_COMMAND_VECTOR:
            self.command_ploter.update_vector(1, self.input_handler.dx, self.input_handler.dy)
            angle = self.input_handler.boat_state_receiver.theta
            dx = 10 * math.cos(angle)
            dy = 10 * math.sin(angle)
            self.command_ploter.update_vector(2, dx, dy)
        pass

    def compute_command(self):
        # Compute the pids output
        # if we are in full speed mode, we don't use the distance pid, we just go full speed
        if self.input_handler.trajectory_receiver.full_speed:
            self.pid_d.u = MAX_THRUST
        else:
            self.pid_d.compute(self.input_handler.e_d, 0.1)
        self.pid_torque.compute(self.input_handler.e_theta, 0.1)
        # Adapt the distance control signal regarding the angle error
        self.pid_d.u *= math.cos(self.input_handler.e_theta)
        if self.pid_d.u < 0:
            self.pid_d.u = THROTLE_FOR_REVERSE
        # log error and control signal and real e d with 2 decimals
        # self.get_logger().info('e_d: "%f" e_theta: "%f" u_d: "%f" u_theta: "%f"' % (
        #     self.input_handler.e_d, self.input_handler.e_theta, self.pid_d.u, self.pid_torque.u))
        if PLOT_COMMAND_VECTOR:
            self.command_ploter.update_vector(0, self.pid_torque.u, self.pid_d.u)

    def main(self):
        # If we are not working, we do nothing
        if not self.working:
            self.control(0, 0)
            return
        # If we are too close to the target, we just try to stop
        if self.input_handler.e_d < MIN_DISTANCE:
            thrust = 10 * self.input_handler.boat_state_receiver.get_speed()
            self.control(thrust, 0)
            return
        # Compute the control signal
        self.compute_command()
        # Send the control signal to the thrusters
        self.control(self.pid_d.u, self.pid_torque.u)

    def controller_mode_cb(self):
        if self.controller_mode_publisher.mode == ControllerMode.DISABLED:
            self.get_logger().info('Controller disabled')
            self.working = False
        elif self.controller_mode_publisher.mode == ControllerMode.ENABLED:
            self.get_logger().info('Controller enabled')
            self.working = True
        else:
            self.get_logger().error('Unknown controller mode')

    def plot_command_timer_cb(self):
        self.command_ploter.update_plot()

    def trajectory_receiver_cb(self):
        self.input_handler.calculate_error()

    def boat_state_receiver_cb(self):
        self.input_handler.calculate_error()

    def control(self, u_d, u_torque):
        self.thrusters.command(u_d, u_torque)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
