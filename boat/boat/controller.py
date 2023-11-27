import rclpy
from rclpy.node import Node
from boat.Target import Target
from boat.BoatState import BoatState

# Topics used by the controller
TOPIC_BOAT_STATUS = '/boat/estimator/position'
TOPIC_NAVIGATOR_TARGET = '/boat/mission/target'

# Topics published by the controller

class Regulator:
    def __init__(self, boatState, target):
        self.boatState = boatState
        self.target = target
        self.kp = 0.1
        self.ki = 0.1
        self.kd = 0.1
        self.integral = 0
        self.previousError = 0
        self.dt = 0.1

    def update(self):
        # Add your regulation logic here
        # self.boatState.updateAll(0, 0, 0, 0, 0)
        # self.target.updateAll(0, 0, 0)
        self.integral += self.boatState.getSpeed() * self.dt
        error = self.target.getOrientation() - self.boatState.getOrientation()
        self.boatState.updateAll(0, 0, 0, 0, 0)
        self.target.updateAll(0, 0, 0)
        return self.kp * error + self.ki * self.integral + self.kd * (error - self.previousError) / self.dt
    

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.boatState = BoatState()
        self.target = Target()
        self.regulator = Regulator(self.boatState, self.target)
        # Subscribers
        self.sub_pos = self.create_subscription(
            BoatState, TOPIC_BOAT_STATUS, self.boatState.updateFromMsg, 10)
        self.sub_target = self.create_subscription(
            Target, TOPIC_NAVIGATOR_TARGET, self.target.updateFromMsg, 10)
        

    def run(self):
        # Add your node logic here
        self.get_logger().info('Controller node is running')

    def timer_callback(self):
        # Add your regulation logic here
        

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    controller.run()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
