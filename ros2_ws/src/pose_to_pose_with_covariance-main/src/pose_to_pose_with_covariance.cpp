#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include "geometry_msgs/msg/pose2_d.hpp"

class PoseToPoseCovStamped : public rclcpp::Node
{
public:
  PoseToPoseCovStamped()
  : Node("pose_to_pose_with_covariance")
  {
    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_cov", 3);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_broadcaster/pose", 1,
      std::bind(&PoseToPoseCovStamped::poseCallback, this, std::placeholders::_1));

    out_msg_.pose.covariance[0] = 3.0;
    out_msg_.pose.covariance[7] = 3.0;
    out_msg_.pose.covariance[14] = 0.00;
    out_msg_.pose.covariance[21] = 0.00;
    out_msg_.pose.covariance[28] = 0.00;
    out_msg_.pose.covariance[35] = 0.0005;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    out_msg_.pose.pose = msg->pose;
    out_msg_.header = msg->header;

    pose_cov_pub_->publish(out_msg_);
  }
  rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    client_node_,
    "navigate_through_poses");
  void handleGoalLoader()
  {
    acummulated_poses_.clear();

    std::cout << "Loading Waypoints!" << std::endl;

    QString file = "/home/nvidia/AVC-Ros/ros2_ws/waypoints.yaml";

    YAML::Node available_waypoints;

    try {
      available_waypoints = YAML::LoadFile(file.toStdString());
    } catch (const std::exception & ex) {
      std::cout << ex.what() << ", please select a valid file" << std::endl;
      updateWpNavigationMarkers();
      return;
    }

    const YAML::Node & waypoint_iter = available_waypoints["waypoints"];
    for (YAML::const_iterator it = waypoint_iter.begin(); it != waypoint_iter.end(); ++it) {
      auto waypoint = waypoint_iter[it->first.as<std::string>()];
      auto pose = waypoint["pose"].as<std::vector<double>>();
      auto orientation = waypoint["orientation"].as<std::vector<double>>();
      acummulated_poses_.push_back(convert_to_msg(pose, orientation));
    }

    // Publishing Waypoint Navigation marker after loading wp's
    //updateWpNavigationMarkers();
  }

  void startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses)
  {
    auto is_action_server_ready =
      nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready) {
      RCLCPP_ERROR(
        client_node_->get_logger(), "navigate_through_poses action server is not available."
        " Is the initial pose set?");
      return;
    }

    nav_through_poses_goal_.poses = poses;
    RCLCPP_INFO(
      client_node_->get_logger(),
      "NavigateThroughPoses will be called using the BT Navigator's default behavior tree.");

    RCLCPP_DEBUG(
      client_node_->get_logger(), "Sending a path of %zu waypoints:",
      nav_through_poses_goal_.poses.size());
    for (auto waypoint : nav_through_poses_goal_.poses) {
      RCLCPP_DEBUG(
        client_node_->get_logger(),
        "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
    }

    // Enable result awareness by providing an empty lambda function
    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {
        nav_through_poses_goal_handle_.reset();
      };

    auto future_goal_handle =
      nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
    if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
      return;
    }

    // Get the goal handle and save so that we can check on completion in the timer callback
    nav_through_poses_goal_handle_ = future_goal_handle.get();
    if (!nav_through_poses_goal_handle_) {
      RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
      return;
    }

    timer_.start(200, this);
  }

protected:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  geometry_msgs::msg::PoseWithCovarianceStamped out_msg_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PoseToPoseCovStamped>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

