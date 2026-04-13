#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

    out_msg_.pose.covariance[0] = 0.05;
    out_msg_.pose.covariance[7] = 0.05;
    out_msg_.pose.covariance[14] = 0.00;
    out_msg_.pose.covariance[21] = 0.00;
    out_msg_.pose.covariance[28] = 0.00;
    out_msg_.pose.covariance[35] = 0.01;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    out_msg_.pose.pose = msg->pose;
    out_msg_.header = msg->header;

    pose_cov_pub_->publish(out_msg_);
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
