#pragma once

/* Gets the current FPGA time */

#define GET_TIME_DOUBLE (frc::Timer::GetFPGATimestamp().to<double>())
#define GET_TIME_INT (frc::Timer::GetFPGATimestamp().to<int>())

/* Unpacks arrays to be put in arguments */

#define UNPACK_2(arr) (arr.at(0)), (arr.at(1))
#define UNPACK_3(arr) UNPACK_2(arr), (arr.at(2))
#define UNPACK_4(arr) UNPACK_3(arr), (arr.at(3))
#define UNPACK_5(arr) UNPACK_4(arr), (arr.at(4))

/* Math functions */

/* Converts a -1:1 range value to be 0:1 */
#define MAKE_SCALAR(v) (((v) + 1) / 2)
/* Checks if a value is within two other values, inclusive */
#define IS_WITHIN(v, min, max) ((v >= min) && (v <= max))

/* ROS functions */

#define ROS_PUB(msg) rclcpp::Publisher<msg>::SharedPtr
#define ROS_SUB(msg) rclcpp::Subscription<msg>::SharedPtr
#define ROS_SERVICE(msg) rclcpp::Service<msg>::SharedPtr
#define ROS_CLIENT(msg) rclcpp::Client<msg>::SharedPtr

/* Common message types */

typedef std_msgs::msg::Int16 IntMsg 
typedef std_msgs::msg::Float32 FloatMsg
typedef std_msgs::msg::Bool BoolMsg
typedef autobt_msgs::srv::StringService StringSrv
typedef sensor_msgs::msg::Joy JoyMsg

/* QoS types */

#define DEFAULT_QOS (rclcpp::SystemDefaultsQoS())
#define SENSOR_QOS (rclcpp::SensorDataQoS())