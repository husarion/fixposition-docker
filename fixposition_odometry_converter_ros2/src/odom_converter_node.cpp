// Port of the fixposition_odometry_converter 
// (https://github.com/fixposition/fixposition_driver/tree/main/fixposition_odometry_converter) to ROS 2

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <fixposition_driver_ros2/msg/speed.hpp>

namespace fixposition
{

class OdomConverterNode : public rclcpp::Node
{
public:
  OdomConverterNode(const rclcpp::NodeOptions & options) : Node("odom_converter", options)
  {
    this->declare_parameter<std::string>("fixposition_speed_topic", "/fixposition/speed");
    this->declare_parameter<int>("multiplicative_factor", 1000);
    this->declare_parameter<bool>("use_angular", false);
    this->declare_parameter<std::string>("input_topic", "/odom");

    std::string fixposition_speed_topic =
      this->get_parameter("fixposition_speed_topic").as_string();
    multiplicative_factor_ = this->get_parameter("multiplicative_factor").as_int();
    use_angular_ = this->get_parameter("use_angular").as_bool();
    std::string input_topic = this->get_parameter("input_topic").as_string();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic, 10, std::bind(&OdomConverterNode::OdomCb, this, std::placeholders::_1));
    speed_pub_ =
      this->create_publisher<fixposition_driver_ros2::msg::Speed>(fixposition_speed_topic, 10);
  }

private:
  void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    fixposition_driver_ros2::msg::Speed speed_msg;
    speed_msg.speeds.push_back(msg->twist.twist.linear.x * multiplicative_factor_);
    if (use_angular_) {
      speed_msg.speeds.push_back(msg->twist.twist.angular.z * multiplicative_factor_);
    }
    speed_pub_->publish(speed_msg);
  }

  int multiplicative_factor_;
  bool use_angular_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<fixposition_driver_ros2::msg::Speed>::SharedPtr speed_pub_;
};

}  // namespace fixposition

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fixposition::OdomConverterNode)