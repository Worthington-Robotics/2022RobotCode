#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autobt_msgs/srv/run_tree.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class AutoSelect {
    public:
    AutoSelect(std::vector<std::string>);

    void createRosBindings(rclcpp::Node *);

    void selectAuto(std::string);

    void saveAuto(const std_msgs::msg::String);
    
    private:

    rclcpp::Client<autobt_msgs::srv::RunTree>::SharedPtr treeRunner;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr delimAutoList;
    std::vector<std::string> autos = {};

};