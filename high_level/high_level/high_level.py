import rclpy
import math
from rclpy.node import Node
from high_level.state_tracker import StateTrackerReceiver, Phase
from high_level.buoy import BuoyReceiver
from high_level.path_finder import PathFinder
from boat.boat_state import BoatStateReceiver
from boat.trajectory import TrajectoryPublisher
from boat.controller_mode import ControllerModePublisher, ControllerMode
from high_level.obstacle import get_obstacles
from high_level.graph_plotter import GraphPlotter
from high_level.threat_info import ThreatInfoReceiver


PLOT_GRAPH = False
PLOT_GRAPH_PERIOD_S = 0.1

THREAT_CLOSE_DISTANCE = 50 # Distance à laquelle on considère que le bateau est proche de la menace
BOY_CLOSE_DISTANCE = 10 # Distance à laquelle on considère que le bateau a atteint la bouée

# we don t aim straight at the buoy, we aim at a point in front of it

BUOY_KEEPOUT = 15

class HighLevel(Node):
    def __init__(self):
        super().__init__('high_level_node')
        self.trajectory_publisher:  TrajectoryPublisher = TrajectoryPublisher(self)
        self.controller_mode_publisher: ControllerModePublisher = ControllerModePublisher(self)
        self.state_tracker_receiver = StateTrackerReceiver(self)
        self.buoy_receiver = BuoyReceiver(self)
        self.threat_info_receiver = ThreatInfoReceiver(self)
        self.boat_state = BoatStateReceiver(self)
        self.path_finder = PathFinder(get_obstacles())
        self.timer = self.create_timer(0.5, self.main)
        self.phase: Phase = Phase.INIT

        self.last_valid_buoy_pos = (0, 0)
        self.buoy_is_valid_graph = False
        self.buoy_path = []
        self.target_path = []

        self.base_lat = 0
        self.base_lon = 0

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
        self.buoy_reached = False

        self.target_is_position_known = False

        self.controller_mode = ControllerMode.DISABLED

    def convert_gps_to_xy(self, latitude, longitude):
        # Haversine distance calculation
        R = 6371000  # Earth radius in meters

        dLat = math.radians(latitude - self.base_lat)
        dLon = math.radians(longitude - self.base_lon)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.sin(dLon / 2) * math.sin(dLon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # Calculate x and y based on the distance and bearing
        y = math.sin(dLon) * math.cos(math.radians(latitude))
        x = math.cos(math.radians(self.base_lat)) * math.sin(math.radians(latitude)) - math.sin(math.radians(self.base_lat)) * math.cos(math.radians(latitude)) * math.cos(dLon)
        bearing = math.atan2(y, x)
        y = distance * math.cos(bearing)
        x = distance * math.sin(bearing)
        return x, y

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
            self.set_controller_mode(ControllerMode.ENABLED)
        pass

    def buoy(self):
        if self.buoy_reached:
            self.set_phase(Phase.PATROL)
            return
        # Get variables
        boat_x, boat_y = self.boat_state.get_pos()
        buoy_x, buoy_y = self.last_valid_buoy_pos
        # correct buoy position
        distance = math.sqrt((buoy_x - boat_x)**2 + (buoy_y - boat_y)**2)
        angle = math.atan2(buoy_y - boat_y, buoy_x - boat_x)
        distance -= BUOY_KEEPOUT
        buoy_x = boat_x + distance * math.cos(angle)
        buoy_y = boat_y + distance * math.sin(angle)
        # Check if we are close enough
        distance = math.sqrt((buoy_x - boat_x)**2 + (buoy_y - boat_y)**2)
        if distance < BOY_CLOSE_DISTANCE:
            self.get_logger().info('Buoy reached')
            self.buoy_reached = True
            self.set_phase(Phase.PATROL)
            return
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
            self.trajectory_publisher.publish(buoy_x, buoy_y, full_speed=False)
        else :
            self.get_logger().info('Path found')
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
        # Log x y target
        self.get_logger().info('Target: %f %f' % (self.buoy_path[1][0], self.buoy_path[1][1]))
        self.trajectory_publisher.publish(self.buoy_path[1][0], self.buoy_path[1][1], full_speed=True)

    def patrol(self):
        self.set_controller_mode(ControllerMode.DISABLED)
        if self.target_is_position_known:
            self.set_phase(Phase.PURSUIT)
        pass

    def set_controller_mode(self, mode: ControllerMode):
        if mode != self.controller_mode_publisher.mode:
            self.controller_mode_publisher.publish(mode)

    def pursuit(self):
        # Are we close to the target?
        target_x, target_y = self.threat_info_receiver.get_pos()
        boat_x, boat_y = self.boat_state.get_pos()
        distance = math.sqrt((target_x - boat_x)**2 + (target_y - boat_y)**2)
        if distance < THREAT_CLOSE_DISTANCE:
            self.set_controller_mode(ControllerMode.DISABLED)
        else:
            self.pursuit_far()

    def pursuit_far(self):
        self.set_controller_mode(ControllerMode.ENABLED)
        target_x, target_y = self.threat_info_receiver.get_pos()
        boat_x, boat_y = self.boat_state.get_pos()
        # use astar
        self.path_finder.set_position(boat_x, boat_y)
        # TODO add moving obstacles
        # Compute path
        path = self.path_finder.find_path((target_x, target_y))
        # no path found
        if len(path) == 0:
            self.get_logger().error('No path found')
            if len(self.target_path) > 0:
                # log using the known target path
                self.get_logger().info('Using known target path')
                self.publish_target_from_target_path()
        elif len(path) == 1:
            # we re too close
            self.get_logger().info('Target reached')
            self.set_controller_mode(ControllerMode.DISABLED)
        else :
            # Save the path for the target
            self.target_path = path
            self.publish_target_from_target_path()
        pass

    def publish_target_from_target_path(self):
        if len(self.target_path) == 0:
            self.get_logger().error('No path found')
            return
        # Log x y target
        self.get_logger().info('Target: %f %f' % (self.target_path[1][0], self.target_path[1][1]))
        self.trajectory_publisher.publish(self.target_path[1][0], self.target_path[1][1], full_speed=True)
    def pursuit_close(self):
        pass

    def threat_info_cb(self):
        # must be implemented
        self.target_is_position_known = True
        pass

    def state_tracker_cb(self):
        self.get_logger().info('State received: %s' % self.state_tracker_receiver.get_phase().name)
        if self.phase == Phase.BUOY:
            self.get_logger().info('Buoy reached')
            if (self.state_tracker_receiver.get_phase() == Phase.PATROL
                or self.state_tracker_receiver.get_phase() == Phase.PURSUIT):
                self.buoy_reached = True
        elif self.phase == Phase.PATROL:
            if self.state_tracker_receiver.get_phase() == Phase.PURSUIT:
                self.target_is_position_known = True

    def buoy_receiver_cb(self):
        if self.path_finder.check_if_target_valid(self.buoy_receiver.get_buoy_pos()):
            if self.phase == Phase.INIT:
                self.buoy_received = True
            self.last_valid_buoy_pos = self.buoy_receiver.get_buoy_pos()
        pass

    def boat_state_receiver_cb(self):
        if self.phase == Phase.INIT:
            self.pos_received = True
        self.base_lat = self.boat_state.base_lat
        self.base_lon = self.boat_state.base_lon
        pass


def main(args=None):
    rclpy.init(args=args)
    highLevel = HighLevel()
    rclpy.spin(highLevel)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
