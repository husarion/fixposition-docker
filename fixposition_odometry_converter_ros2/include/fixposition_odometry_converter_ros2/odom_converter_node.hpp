// Port of the fixposition_odometry_converter 
// (https://github.com/fixposition/fixposition_driver/tree/main/fixposition_odometry_converter) to ROS 2

#ifndef __ODOM_CONVERTER_HPP__
#define __ODOM_CONVERTER_HPP__

/* EXTERNAL */

/* ROS */
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/* PACKAGE */
#include <fixposition_driver_ros2/msg/speed.hpp>
#include <fixposition_odometry_converter_ros2/params.hpp>

namespace fixposition {

class OdomConverterNode : public rclcpp::Node {
   public:
    /**
     * @brief Construct a new OdomConverter object
     *
     * @param[in] nh node handle
     */
    OdomConverterNode(const rclcpp::NodeOptions& options);

    /**
     * @brief Destroy the OdomConverter object
     *
     */
    ~OdomConverterNode() = default;

    /**
     * @brief Subscribes to the correct topic depending on the parameters
     *
     */
    void Subscribe();

    /**
     * @brief Converts and publishes the speed to an integer value in [mm/s]
     *
     * @param speed
     */
    void ConvertAndPublish(const double speed, const double angular, bool use_angular = false);

   private:
    /**
     * @brief Converts a message of type TwistWithCovariance to the fixposition_driver Speed message
     *
     * @param[in] msg
     */
    void TwistWithCovCallback(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg);

    /**
     * @brief Converts a message of type Odometry to the fixposition_driver Speed message
     *
     * @param[in] msg
     */
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Converts a message of type Twist to the fixposition_driver Speed message
     *
     * @param[in] msg
     */
    void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    OdomInputParams params_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovariance>::SharedPtr twist_with_covariance_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    rclcpp::Publisher<fixposition_driver_ros2::msg::Speed>::SharedPtr speed_pub_;
};
}  // namespace fixposition
#endif  //__FIXPOSITION_DRIVER_FIXPOSITION_DRIVER__
