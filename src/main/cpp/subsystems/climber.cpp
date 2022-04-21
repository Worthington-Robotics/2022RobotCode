#include "subsystems/climber.h"

using std::placeholders::_1;

namespace robot {
    
    Climber::Climber() {}

    void Climber::createRosBindings(rclcpp::Node* Node) {
        soleStatePub = Node->create_publisher<MSG_INT>("externIO/climber_r_main_solenoid/state", DEFAULT_QOS);
        stick1Sub = Node->create_subscription<MSG_JOY>("sticks/stick1", SENSOR_QOS, std::bind(&Climber::setStick1Input, this, _1));
        climberDemandsPubs = {
            Node->create_publisher<can_msgs::msg::MotorMsg>("externIO/climber_l_motor/demand", DEFAULT_QOS),
            Node->create_publisher<can_msgs::msg::MotorMsg>("externIO/climber_r_motor/demand", DEFAULT_QOS)
        };
    }

    void Climber::publishData() {
        for (int i = 0; i < (int)(climberEnabled.size()); i++) {
            can_msgs::msg::MotorMsg demands;
            demands.arb_feedforward = 0;
            demands.control_mode = 0;
            if (climberEnabled.at(i)) {
                demands.demand = climberDemands.at(i);
            } else {
                demands.demand = 0;
            }
            climberDemandsPubs.at(i)->publish(demands);
        }
        if (enableSole) {
            if (!solePressed) {
                soleState = !soleState;
                solePressed = true;
            }
        } else {
            solePressed = false;
        }
        MSG_INT soleDemand;
        soleDemand.data = -1;
        if (soleState) {
            soleDemand.data = 1;
        }
        soleStatePub->publish(soleDemand);
    }
} // namespace robot