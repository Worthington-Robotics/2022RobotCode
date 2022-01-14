#include "subsystems/LightBar.h"

#include <iostream>

using std::placeholders::_1;

namespace robot {
    LightBar::LightBar(): leds{LED_BAR}{
        leds.SetLength(ledCount);
        LightBar::reset();
        std::cout << "Made Lightbar";
    }
    void LightBar::createRosBindings(rclcpp::Node *node) {
        LightModeSub = node->create_subscription<std_msgs::msg::Int16>("/lights/mode", rclcpp::SensorDataQoS(), std::bind(&LightBar::lightModeCallback, this, _1));
    }
    void LightBar::reset() {
        leds.Start();
    }
    void LightBar::onStart() {

    }
    void LightBar::onLoop() {
        
    }
    void LightBar::publishData() {
        for (int i = 0; i < ledCount; i++) {
            buffer[i].SetLED(LightBar::getColor(i));
        }
        step++;
        leds.SetData(buffer);
    }
    frc::Color LightBar::getColor(int pos) {
        double por = (double)pos / (double)ledCount;
        switch (lightMode) {
            case RAINBOW: 
                int speed = 100;
                double scale = ((double)(step % speed) / (double)(speed - 1));
                double h = (por + scale - floor(por + scale)) * 255.0;
                return frc::Color::FromHSV((int)h, 255, 230);
            case TEST:
                if (true) {
                    return frc::Color::FromRGB(0, 255, 0);
                }
                return frc::Color::FromRGB(255, 0, 0);
            case ALLIANCE:
                bool pattern [17] = {true, true, true, true, false, true, false, true, true, true, true, false, true, true, true, true, true}
        }
    }
    void LightBar::lightModeCallback(const std_msgs::msg::Int16 msg) {
        std::cout << "changing lights to mode " << msg.data << std::endl;
        lightMode = static_cast<LightState>(msg.data);
    }
    void LightBar::updateSensorData() {

    }
}
