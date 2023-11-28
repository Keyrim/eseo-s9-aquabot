import rclpy
from rclpy.node import Node
from boat.Target import Target
from boat.BoatState import BoatStateReceiver


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.boat_state_receiver = BoatStateReceiver(self)
        

    def run(self):
        # Add your node logic here
        self.get_logger().info('Controller node is running')

    def timer_callback(self):
        # Add your regulation logic here
        pass
    
    def boat_state_receiver_cb(self):
        # Add your regulation logic here
        pass
    
    
        

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    controller.run()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
