import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node

 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module

from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory
import os
import yaml
 
class waypointstarterNode(Node):
    def __init__(self):
        super().__init__('waypointstarter')

        # Set up subscriber and publisher nodes
        # the "self.joy_callback" means that upon receiving a message from the topic, ->
        # the function "joy_callback" will be called AUTOMATICALLY with the message data as its argument
        self.subscription_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        waypoints_dir = get_package_share_directory('waypointstarter') 
        parameters_file_dir = os.path.join(waypoints_dir, 'data') 
        parameters_file_path = os.path.join(parameters_file_dir, 'goalstest.yaml')
        
        self.navigator=BasicNavigator()
        self.waypoint_list = []
        with open(parameters_file_path, 'r') as file:
            waypoints=yaml.safe_load(file)
        for key, value in waypoints['waypoints'].items():
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            pose.pose.position.x = value['pose'][0]
            pose.pose.position.y = value['pose'][1]
            pose.pose.position.z = value['pose'][2]
            
            pose.pose.orientation.x = value['orientation'][0]
            pose.pose.orientation.y = value['orientation'][1]
            pose.pose.orientation.z = value['orientation'][2]
            pose.pose.orientation.w = value['orientation'][3]
            
            self.waypoint_list.append(pose)
      
    def joy_callback(self, data):
        if data.buttons[0]==1: # TODO: change to whatever button press is; if the joy says start, then send the waypoints
            # Start the ROS 2 Python Client Library
            # rclpy.init()
            self.get_logger().info('IT WORKSSSS')
            # Launch the ROS 2 Navigation Stack
            #navigator = BasicNavigator()
            self.navigator.followWaypoints(self.waypoint_list)

            self.navigator.lifecycleShutdown()
            exit(0)
		
        else: # else, do nothing
            return 

def main(args=None):
    rclpy.init(args=args)

    waypointstarter = waypointstarterNode()

    rclpy.spin(waypointstarter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    waypointstarter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
