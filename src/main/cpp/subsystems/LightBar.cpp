#include "subsystems/LightBar.h"

namespace robot {
    LightBar::LightBar() {

    }
    void LightBar::reset() {

    }
    void LightBar::onStart() {

    }
    void LightBar::onLoop() {

    }
    void LightBar::publishData() {
        
    }
    void LightBar::lightModeCallback(const std_msgs::msg::Int16 msg)
    {
        std::cout << "changing lights to mode " << msg.data << std::endl;
        lightState = static_cast<LightState>(msg.data);
    }
}