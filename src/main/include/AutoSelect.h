#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autobt_msgs/srv/run_tree.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>

class AutoSelect {
    public:
    AutoSelect(std::vector<std::string>);

    void createRosBindings(rclcpp::Node *);

    void selectAuto(std::string);

    void saveAuto(const std_msgs::msg::String);
    
    private:

    rclcpp::Client<autobt_msgs::srv::RunTree>::SharedPtr treeRunner;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr delimAutoListSub;
    std::vector<std::string> autos = {};
    bool isAutoLoaded = false;

};