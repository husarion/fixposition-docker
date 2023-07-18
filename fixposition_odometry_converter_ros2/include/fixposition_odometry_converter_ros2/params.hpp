// Port of the fixposition_odometry_converter 
// (https://github.com/fixposition/fixposition_driver/tree/main/fixposition_odometry_converter) to ROS 2

#ifndef __ODOM_CONVERTER_PARAMS_HPP__
#define __ODOM_CONVERTER_PARAMS_HPP__

/* SYSTEM / STL */
#include <string>

/* EXTERNAL */
#include <rclcpp/rclcpp.hpp>

/* PACKAGE */

namespace fixposition {

enum class VelTopicType : int8_t { Twist = 0, TwistWithCov = 1, Odometry = 2 };

struct OdomInputParams {
    VelTopicType topic_type;
    std::string input_topic;
    std::string fixposition_speed_topic;
    int multiplicative_factor;
    bool use_angular;
    /**
     * @brief Load all parameters from ROS parameter server
     *
     * @param[in] ns namespace to load the parameters from
     * @return true success
     * @return false fail
     */
    bool LoadFromRos(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& param_itf,
                     const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_itf);
};

}  // namespace fixposition

#endif
