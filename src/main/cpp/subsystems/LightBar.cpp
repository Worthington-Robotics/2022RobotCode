#include "subsystems/LightBar.h"

using std::placeholders::_1;

namespace robot {
    LightBar::LightBar(): leds{LED_BAR}{
        leds.SetLength(ledCount);
        LightBar::reset();
    }
    void LightBar::createRosBindings(rclcpp::Node *node) {
        DriveModeSub = node->create_subscription<std_msgs::msg::Int16>("/lights/mode", rclcpp::SensorDataQoS(), std::bind(&LightBar::lightModeCallback, this, _1));
    }
    void LightBar::reset() {
        leds.Start();
    }
    void LightBar::onStart() {

    }
    void LightBar::onLoop() {
        for (int i = 0; i < ledCount; i++) {
            buffer[i].SetLED(LightBar::getColor(i));
        }
    }
    void LightBar::publishData() {
        leds.SetData(buffer);
    }
    frc::Color LightBar::getColor(int pos) {
        double por = (double)pos / (double)ledCount;
        switch (lightMode) {
            case RAINBOW: 
                int speed = 500;
                int cycle = frc::Timer::GetFPGATimestamp().to<int>();
                double scale = (double)(cycle % speed) / (double)(speed - 1);
                double h = (por + scale - floor(por + scale)) * 255.0;
                return frc::Color::FromHSV((int)h, 255, 230);
        }
    }
    void LightBar::lightModeCallback(const std_msgs::msg::Int16 msg) {
        std::cout << "changing lights to mode " << msg.data << std::endl;
        lightMode = static_cast<LightState>(msg.data);
    }
}