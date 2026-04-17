import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, SendGoalOptions

 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
from nav2_msgs.action import NavigateThroughPoses

from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler
import os
import yaml

 
class waypointstarterNode(Node):
    def __init__(self):
        super().__init__('waypointstarter')

        # Set up subscriber and publisher nodes
        # the "self.joy_callback" means that upon receiving a message from the topic, ->
        # the function "joy_callback" will be called AUTOMATICALLY with the message data as its argument
        self.subscription_joy = self.create_subscription(Joy, '/joy_start', self.joy_callback, 10)

        waypoints_dir = get_package_share_directory('waypointstarter') 
        parameters_file_dir = os.path.join(waypoints_dir, 'data') 
        parameters_file_path = os.path.join(parameters_file_dir, 'noramp.yaml')

        self.waypoint_list = []
        self.navigator=BasicNavigator()
        self.nav_through_poses_action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.nav_through_poses_goal = NavigateThroughPoses.Goal()
        self.nav_through_poses_goal_handle = None
        self.server_timeout = Duration(seconds=5)
        self.timer = None
        with open(parameters_file_path, 'r') as file:
            waypoints=yaml.safe_load(file)

        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 1.0
        # self.navigator.setInitialPose(initial_pose)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        
        x_, y_, z_, w_, = quaternion_from_euler(0.0, 0.0, 0.0)
        pose.pose.orientation.x = x_
        pose.pose.orientation.y = y_
        pose.pose.orientation.z = z_
        pose.pose.orientation.w = w_
        
        self.navigator.setInitialPose(pose)

        for key, value in waypoints['waypoints'].items():
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            pose.pose.position.x = float(value['pose'][0])
            pose.pose.position.y = float(value['pose'][1])
            pose.pose.position.z = float(value['pose'][2])
            
            pose.pose.orientation.x = float(value['orientation'][2])
            pose.pose.orientation.y = float(value['orientation'][1])
            pose.pose.orientation.z = float(value['orientation'][0])
            pose.pose.orientation.w = float(value['orientation'][3])
            
            self.waypoint_list.append(pose)
      
    def startNavThroughPoses(self, poses):
        is_action_server_ready = self.nav_through_poses_action_client.wait_for_server(timeout_sec=5.0)
        if not is_action_server_ready:
            self.get_logger().error(
                "navigate_through_poses action server is not available. Is the initial pose set?"
            )
            return

        self.nav_through_poses_goal.poses = poses
        self.get_logger().info(
            "NavigateThroughPoses will be called using the BT Navigator's default behavior tree."
        )

        self.get_logger().debug(
            f"Sending a path of {len(self.nav_through_poses_goal.poses)} waypoints:"
        )
        for waypoint in self.nav_through_poses_goal.poses:
            self.get_logger().debug(
                f"\t({waypoint.pose.position.x}, {waypoint.pose.position.y})"
            )

        # Enable result awareness by providing an empty lambda function
        send_goal_options = SendGoalOptions()
        send_goal_options.result_callback = lambda result: setattr(self, 'nav_through_poses_goal_handle', None)

        future_goal_handle = self.nav_through_poses_action_client.send_goal_async(
            self.nav_through_poses_goal, send_goal_options
        )
        if rclpy.spin_until_future_complete(self, future_goal_handle, self.server_timeout) != rclpy.FutureReturnCode.SUCCESS:
            self.get_logger().error("Send goal call failed")
            return

        # Get the goal handle and save so that we can check on completion in the timer callback
        self.nav_through_poses_goal_handle = future_goal_handle.result()
        if not self.nav_through_poses_goal_handle:
            self.get_logger().error("Goal was rejected by server")
            return

        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        if self.nav_through_poses_goal_handle and self.nav_through_poses_goal_handle.done():
            self.nav_through_poses_goal_handle = None
            if self.timer:
                self.timer.cancel()
                self.timer = None
      
    def joy_callback(self, data):
        if data.buttons[0]==1: # TODO: change to whatever button press is; if the joy says start, then send the waypoints
            # Start the ROS 2 Python Client Library
            #rclpy.init()
            
            # Launch the ROS 2 Navigation Stack
            # navigator = BasicNavigator()
            #self.navigator.followWaypoints(self.waypoint_list)
            self.startNavThroughPoses(self.waypoint_list)
            #self.navigator.lifecycleShutdown()
            exit(0)
    
		
        else: # else, do nothing
            return 

def main(args=None):
    rclpy.init(args=args)

    waypointstarter = waypointstarterNode()

    #rclpy.spin(waypointstarter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    #waypointstarter.destroy_node()

    #rclpy.shutdown()
    
    executor = MultiThreadedExecutor()
    executor.add_node(waypointstarter)
    
    try:
        executor.spin()
    finally:
        waypointstarter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
