// Port of the fixposition_odometry_converter 
// (https://github.com/fixposition/fixposition_driver/tree/main/fixposition_odometry_converter) to ROS 2

#include <fixposition_odometry_converter_ros2/odom_converter_node.hpp>

namespace fixposition {

OdomConverterNode::OdomConverterNode(const rclcpp::NodeOptions& options) : Node("odom_converter", options) {
    // read parameters
    if (!params_.LoadFromRos(this->get_node_parameters_interface(), this->get_node_logging_interface())) {
        RCLCPP_ERROR(this->get_logger(), "Parameter Loading failed, shutting down...");
        rclcpp::shutdown();
    }

    // initialize the subscriber
    Subscribe();

    // initialize the publisher
    speed_pub_ = this->create_publisher<fixposition_driver_ros2::msg::Speed>(params_.fixposition_speed_topic, 10);
}

void OdomConverterNode::Subscribe() {
    switch (params_.topic_type) {
        case VelTopicType::Twist:
            twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                params_.input_topic, 10, std::bind(&OdomConverterNode::TwistCallback, this, std::placeholders::_1));

            break;
        case VelTopicType::TwistWithCov:
            twist_with_covariance_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovariance>(
                params_.input_topic, 10,
                std::bind(&OdomConverterNode::TwistWithCovCallback, this, std::placeholders::_1));
            break;
        case VelTopicType::Odometry:
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                params_.input_topic, 10, std::bind(&OdomConverterNode::OdometryCallback, this, std::placeholders::_1));
            break;
    }
}

void OdomConverterNode::ConvertAndPublish(const double speed, const double angular, bool use_angular) {
    if (speed_pub_->get_subscription_count() > 0) {
        const int int_speed = round(speed * params_.multiplicative_factor);
        const int angular_speed = round(angular * params_.multiplicative_factor);
        fixposition_driver_ros2::msg::Speed msg;
        msg.speeds.push_back(int_speed);
        if (params_.use_angular) {
            msg.speeds.push_back(angular_speed);
        }
        speed_pub_->publish(msg);
    }
}

void OdomConverterNode::TwistWithCovCallback(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg) {
    ConvertAndPublish(msg->twist.linear.x, msg->twist.angular.z, params_.use_angular);
}

void OdomConverterNode::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ConvertAndPublish(msg->twist.twist.linear.x, msg->twist.twist.angular.z, params_.use_angular);
}

void OdomConverterNode::TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    ConvertAndPublish(msg->linear.x, msg->angular.z, params_.use_angular);
}

}  // namespace fixposition

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fixposition::OdomConverterNode)