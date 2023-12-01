import rclpy
from rclpy.node import Node
from high_level.state_tracker import StateTrackerReceiver, Phase
from high_level.buoy import BuoyReceiver
from high_level.path_finder import PathFinder
from boat.boat_state import BoatStateReceiver
from boat.trajectory import TrajectoryPublisher
from high_level.obstacle import get_obstacles


class HighLevel(Node):
    def __init__(self):
        super().__init__('high_level_node')
        self.trajectory_publisher = TrajectoryPublisher(self)
        self.state_tracker_receiver = StateTrackerReceiver(self)
        self.buoy_receiver = BuoyReceiver(self)
        self.boat_state = BoatStateReceiver(self)
        self.path_finder = PathFinder(get_obstacles())
        self.timer = self.create_timer(0.5, self.main)
        self.phase: Phase = Phase.INIT

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
        pass

    def buoy(self):
        # Get variables
        buoy_x, buoy_y = self.buoy_receiver.get_buoy_pos()
        boat_x, boat_y = self.boat_state.get_pos()
        # Config path finder
        self.path_finder.set_position(boat_x, boat_y)
        # TODO add moving obstacles
        # Compute path
        path = self.path_finder.find_path(buoy_x, buoy_y)
        # Send the first point of the path to the controller
        self.trajectory_publisher.publish(path[1][0], path[1][1])

    def patrol(self):
        pass

    def pursuit(self):
        pass

    def state_tracker_cb(self):
        # Must be provided
        pass

    def buoy_cb(self):
        if self.phase == Phase.INIT:
            self.buoy_received = True
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
