import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from global_racetrajectory_optimization import main_globaltraj

class RacelineGenerator(Node):
    def __init__(self):
        super().__init__('raceline_generator')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.subscription


    def map_callback(self, msg):
        main_globaltraj.main()
        
        
        

def main(args=None):
    rclpy.init(args=args)
    raceline_generator = RacelineGenerator()
    rclpy.spin(raceline_generator)
    raceline_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()