import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, PoseStamped

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.subscription = self.create_subscription(
            Point, 
            'team2/goal1',
            self.point_callback,
            10
        )

        
    def point_callback(self, point):
        goal_pose = PoseStamped()

        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = point.x
        goal_pose.pose.position.y = point.y
        goal_pose.pose.postition.z = point.z

        #needs an orientation? (quaternion)
        #goal_pose.pose.orientation.y

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        