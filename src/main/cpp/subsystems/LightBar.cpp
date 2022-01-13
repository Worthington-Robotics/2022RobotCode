#include "subsystems/LightBar.h"

using std::placeholders::_1;

namespace robot {
    LightBar::LightBar(): leds{9}{
    }
    void LightBar::createRosBindings(rclcpp::Node *node) {
        DriveModeSub = node->create_subscription<std_msgs::msg::Int16>("/lights/mode", rclcpp::SensorDataQoS(), std::bind(&LightBar::lightModeCallback, this, _1));
    }
    void LightBar::reset() {

    }
    void LightBar::onStart() {

    }
    void LightBar::onLoop() {

    }
    void LightBar::publishData() {
        
    }
    void LightBar::lightModeCallback(const std_msgs::msg::Int16 msg) {
        std::cout << "changing lights to mode " << msg.data << std::endl;
        lightState = static_cast<LightState>(msg.data);
    }
}