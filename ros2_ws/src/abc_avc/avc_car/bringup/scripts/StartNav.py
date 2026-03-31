import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node

 
from robot_navigator import BasicNavigator, NavigationResult # Helper module

from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy
 
class StartNav(Node):
    def __init__(self):
        super().__init__('StartNav')

        # Set up subscriber and publisher nodes
        # the "self.joy_callback" means that upon receiving a message from the topic, ->
        # the function "joy_callback" will be called AUTOMATICALLY with the message data as its argument
        self.subscription_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
      
    def joy_callback(self, data):
        if XXX: # if the joy says start, then send the waypoints
            # Start the ROS 2 Python Client Library
            rclpy.init()
            
            # Launch the ROS 2 Navigation Stack
            navigator = BasicNavigator()

            nav_start = navigator.get_clock().now()
            navigator.followWaypoints(goal_poses)
        else: # else, do nothing
            return 
        