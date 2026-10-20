import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

class Goal1Publisher(Node):

    def __init__(self):
        super().__init__('goal1publisher')
        self.publisher_ = self.create_publisher(Point, 'team2/goal1', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = Point()
        msg.x = 1.0
        msg.y = 1.0
        msg.z = 1.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    goal1publisher = Goal1Publisher()

    rclpy.spin(goal1publisher)

    goal1publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()