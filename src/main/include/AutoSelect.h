#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autobt_msgs/srv/run_tree.hpp>
#include <string>

class AutoSelect {
    public:
    AutoSelect(std::vector<std::string>);

    void createRosBindings(rclcpp::Node *);

    void selectAuto(std::string);
    
    private:

    rclcpp::Client<autobt_msgs::srv::RunTree>::SharedPtr treeRunner;
    std::vector<std::string> autos = {};

};