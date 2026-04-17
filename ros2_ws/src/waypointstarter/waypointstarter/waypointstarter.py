import yaml
import rclpy
import os
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory

class NavJoyController(Node):
    def __init__(self):
        super().__init__('nav_joy_controller')
        
        # 1. Action Clients
        self._nav_poses_client = ActionClient(
            self, 
            NavigateThroughPoses, 
            'navigate_through_poses'
        )

        self._follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )
        
        # 2. Subscriber for Joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy_start',
            self.joy_callback,
            10
        )

        waypoints_dir = get_package_share_directory('waypointstarter') 
        parameters_file_path = os.path.join(waypoints_dir, 'data', 'noramp.yaml')

        self.accumulated_poses = []
        self.yaml_path = parameters_file_path

    def joy_callback(self, msg):
        # Button 0 (e.g., 'A' or 'X'): Navigate Through Poses
        if msg.buttons[0] == 1:
            self.get_logger().info('Button 0: Starting Nav Through Poses...')
            self.handle_goal_loader()
            if self.accumulated_poses:
                self.send_goal(NavigateThroughPoses, self._nav_poses_client)
            else:
                self.get_logger().warn('No poses found.')

        # Button 1 (e.g., 'B' or 'O'): Follow Waypoints
        elif msg.buttons[1] == 1:
            self.get_logger().info('Button 1: Starting Waypoint Following...')
            self.handle_goal_loader()
            if self.accumulated_poses:
                self.send_goal(FollowWaypoints, self._follow_waypoints_client)
            else:
                self.get_logger().warn('No poses found.')

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
                
            self.get_logger().info(f'Loaded {len(self.accumulated_poses)} waypoints.')
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {str(e)}")

    def convert_to_msg(self, pose, orientation):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pose)
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = map(float, orientation)
        return msg

    def send_goal(self, action_type, client):
        """Generic goal sender for both Nav2 action types."""
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available!")
            return

        # Prepare timestamps
        current_time = self.get_clock().now().to_msg()
        for p in self.accumulated_poses:
            p.header.stamp = current_time
            if p.pose.orientation.w == 0.0 and p.pose.orientation.x == 0.0:
                p.pose.orientation.w = 1.0

        goal_msg = action_type.Goal()
        goal_msg.poses = self.accumulated_poses

        self.get_logger().info(f"Sending goal to {action_type.__name__}...")
        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        
        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_logger().info("Mission Completed"))

def main(args=None):
    rclpy.init(args=args)
    node = NavJoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()