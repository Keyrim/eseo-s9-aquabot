import rclpy
from rclpy.node import Node
from high_level.state_tracker import StateTrackerReceiver, Phase
from high_level.buoy import BuoyReceiver
from high_level.path_finder import PathFinder
from boat.boat_state import BoatStateReceiver
from boat.trajectory import TrajectoryPublisher
from boat.controller_mode import ControllerModePublisher, ControllerMode
from high_level.obstacle import get_obstacles
from high_level.graph_plotter import GraphPlotter


PLOT_GRAPH = False
PLOT_GRAPH_PERIOD_S = 0.1


class HighLevel(Node):
    def __init__(self):
        super().__init__('high_level_node')
        self.trajectory_publisher = TrajectoryPublisher(self)
        self.controller_mode_publisher = ControllerModePublisher(self)
        self.state_tracker_receiver = StateTrackerReceiver(self)
        self.buoy_receiver = BuoyReceiver(self)
        self.boat_state = BoatStateReceiver(self)
        self.path_finder = PathFinder(get_obstacles())
        self.timer = self.create_timer(0.5, self.main)
        self.phase: Phase = Phase.INIT

        self.last_valid_buoy_pos = (0, 0)
        self.buoy_is_valid_graph = False
        self.buoy_path = []

        if PLOT_GRAPH:
            self.graph_plotter = GraphPlotter()
            self.timer_plot_graph = self.create_timer(
                PLOT_GRAPH_PERIOD_S,
                self.graph_plotter.update_plot
            )
            self.graph_plotter.show()
            self.graph_plotter.set_points([(0, 0)])
            self.graph_plotter.update_plot()

        # Logic flags
        self.buoy_received = False
        self.pos_received = False

    def set_phase(self, phase: Phase):
        #Log new current phase
        self.get_logger().info('New phase: %s' % phase.name)
        self.phase = phase

    def main(self):
        if self.phase == Phase.INIT:
            self.init()
        elif self.phase == Phase.BUOY:
            self.buoy()
        elif self.phase == Phase.PATROL:
            self.patrol()
        elif self.phase == Phase.PURSUIT:
            self.pursuit()
        else:
            self.get_logger().error('Unknown phase')
        pass

    def init(self):
        if self.buoy_received and self.pos_received:
            self.set_phase(Phase.BUOY)
            self.controller_mode_publisher.publish(ControllerMode.ENABLED)
        pass

    def buoy(self):
        # Get variables
        boat_x, boat_y = self.boat_state.get_pos()
        buoy_x, buoy_y = self.last_valid_buoy_pos
        # Config path finder
        self.path_finder.set_position(boat_x, boat_y)
        # TODO add moving obstacles
        # Compute path
        path = self.path_finder.find_path((buoy_x, buoy_y))
        # no path found
        if len(path) == 0:
            self.get_logger().error('No path found')
            if self.buoy_is_valid_graph:
                self.publish_target_from_buoy_path()
            if PLOT_GRAPH:
                self.graph_plotter.set_points([(0, 0)])

        elif len(path) == 1:
            self.get_logger().info('Target reached')
            if PLOT_GRAPH:
                self.graph_plotter.set_points([(0, 0)])
            self.trajectory_publisher.publish(buoy_x, buoy_y)
        else :
            if PLOT_GRAPH:
                self.graph_plotter.set_points(path)
            # Save the path for the buoyz
            self.buoy_is_valid_graph = True
            self.buoy_path = path
            self.publish_target_from_buoy_path()

    def publish_target_from_buoy_path(self):
        if len(self.buoy_path) == 0:
            self.get_logger().error('No path found')
            return
        self.trajectory_publisher.publish(self.buoy_path[1][0], self.buoy_path[1][1])

    def patrol(self):
        pass

    def pursuit(self):
        pass

    def state_tracker_cb(self):
        # Must be provided
        pass

    def buoy_receiver_cb(self):
        if self.path_finder.check_if_target_valid(self.buoy_receiver.get_buoy_pos()):
            if self.phase == Phase.INIT:
                self.buoy_received = True
            self.last_valid_buoy_pos = self.buoy_receiver.get_buoy_pos()
        pass

    def boat_state_receiver_cb(self):
        if self.phase == Phase.INIT:
            self.pos_received = True
        pass


def main(args=None):
    rclpy.init(args=args)
    highLevel = HighLevel()
    rclpy.spin(highLevel)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
