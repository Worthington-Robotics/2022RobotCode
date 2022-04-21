#pragma once

#include "Util.h"

#include <rclcpp/rclcpp.hpp>
#include <autobt_msgs/srv/run_tree.hpp>
#include <std_msgs/msg/string.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

#include <string>

class AutoSelect {
public:
    AutoSelect(std::vector<std::string>);

    void createRosBindings(rclcpp::Node *);

    void selectAuto(std::string);

    void saveAuto(const std_msgs::msg::String);
    
private:

    ROS_CLIENT(autobt_msgs::srv::RunTree) treeRunner;
    ROS_SUB(std_msgs::msg::String) delimAutoListSub;
    std::vector<std::string> autos = {};
    bool isAutoLoaded = false;
};