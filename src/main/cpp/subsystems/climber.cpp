#include "subsystems/climber.h"

using std::placeholders::_1;
namespace robot
{
    Climber::Climber()
    {
    }

    void Climber::createRosBindings(rclcpp::Node* Node)
    {
        soleStatePub = Node->create_publisher<std_msgs::msg::Int16>("externIO/climber_r_main_solenoid/state", rclcpp::SystemDefaultsQoS());
        stick1Sub = Node->create_subscription<sensor_msgs::msg::Joy>("sticks/stick1", rclcpp::SensorDataQoS(), std::bind(&Climber::setStick1Input, this, _1));
        climberDemandsPubs = {
            Node->create_publisher<can_msgs::msg::MotorMsg>("externIO/climber_l_motor/demand", rclcpp::SystemDefaultsQoS()),
            Node->create_publisher<can_msgs::msg::MotorMsg>("externIO/climber_r_motor/demand", rclcpp::SystemDefaultsQoS())
        };
    }

    void Climber::publishData(){
        for(int i = 0; i < (int)(climberEnabled.size()); i++){
            can_msgs::msg::MotorMsg demands;
            demands.arb_feedforward = 0;
            demands.control_mode = 0;
            if(climberEnabled.at(i)){
                demands.demand = climberDemands.at(i);
            } else {
                demands.demand = 0;
            }
            climberDemandsPubs.at(i)->publish(demands);
        }
        if (enableSole && !solePressed)
        {
            soleState = !soleState;
            solePressed = true;
        }
        else if (!enableSole)
        {
            solePressed = false;
        }
        std_msgs::msg::Int16 soleDemand;
        soleDemand.data = -1;
        if (soleState)
        {
            soleDemand.data = 1;
        }
        soleStatePub->publish(soleDemand);
    }
}