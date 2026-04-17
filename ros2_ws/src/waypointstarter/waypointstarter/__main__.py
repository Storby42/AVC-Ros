import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy  # Added for joystick support

import os

from ament_index_python.packages import get_package_share_directory

class NavJoyController(Node):
    def __init__(self):
        super().__init__('nav_joy_controller')
        
        # 1. Action Client for Nav2
        self._action_client = ActionClient(
            self, 
            NavigateThroughPoses, 
            'navigate_through_poses'
        )
        
        # 2. Subscriber for Joy topic
        # Change 'joy' to your actual topic name if different
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        

        waypoints_dir = get_package_share_directory('waypointstarter') 
        parameters_file_dir = os.path.join(waypoints_dir, 'data') 
        parameters_file_path = os.path.join(parameters_file_dir, 'noramp.yaml')

        self.accumulated_poses = []
        self._goal_handle = None
        self.yaml_path = parameters_file_path

    def joy_callback(self, msg):
        """
        Triggered every time joystick state changes.
        msg.buttons[0] refers to the first button on the controller.
        """
        # Check if button 0 is pressed
        if msg.buttons[0] == 1:
            self.get_logger().info('Joy button pressed! Starting navigation sequence...')
            
            # 1. Load the poses
            self.handle_goal_loader()
            
            # 2. If poses were loaded, start navigation
            if self.accumulated_poses:
                self.start_nav_through_poses(self.accumulated_poses)
            else:
                self.get_logger().warn('No poses found in YAML to navigate to.')

    def handle_goal_loader(self):
        self.accumulated_poses.clear()
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            waypoint_data = data.get('waypoints', {})
            for key in sorted(waypoint_data.keys()):
                wp = waypoint_data[key]
                msg = self.convert_to_msg(wp['pose'], wp['orientation'])
                self.accumulated_poses.append(msg)
                
            self.get_logger().info(f'Successfully loaded {len(self.accumulated_poses)} waypoints.')
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {str(e)}")

    def convert_to_msg(self, pose, orientation):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pose)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, orientation)
        return msg

    def start_nav_through_poses(self, poses):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        
        self.get_logger().info("Goal accepted, moving...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_logger().info("Navigation Finished"))

def main(args=None):
    rclpy.init(args=args)
    node = NavJoyController()
    
    try:
        # This 'spin' keeps the node alive and listening for joy messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#comment for git push