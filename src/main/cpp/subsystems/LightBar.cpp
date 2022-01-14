#include "subsystems/LightBar.h"

#include <iostream>

using std::placeholders::_1;

namespace robot {
    LightBar::LightBar(): leds{LED_BAR}{
        leds.SetLength(ledCount);
        LightBar::reset();
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
                int speed = 130;
                double scale = ((double)(step % speed) / (double)(speed - 1));
                double h = (por + scale - floor(por + scale)) * 255.0;
                return frc::Color::FromHSV((int)h, 255, 230);
            case TEST:
                if (true) {
                    return frc::Color::FromRGB(0, 255, 0);
                }
                return frc::Color::FromRGB(255, 0, 0);
            case ALLIANCE:
                frc::Color allianceColor;
                switch (frc::DriverStation::GetAlliance()) {
                    case kBlue:
                        allianceColor = frc::Color::FromRGB(0, 0, 255);
                    case kRed:
                        allianceColor = frc::Color::FromRGB(255, 0, 0);
                }
                if (ledCount < 17) {return allianceColor;}
                bool pattern [17] = {true, true, true, true, false, true, false, true, true, true, true, false, true, true, true, true, true};
                int margin = (ledCount - 17) / 2;
                if (pos < margin || pos >= margin + 17) {
                    return allianceColor;
                }
                if (pattern[pos - margin]) {
                    return frc::Color::FromRGB(255, 255, 255);
                }
                return allianceColor;
        }
    }
    frc::Color meterColor(int pos, double value) {
        int portion = (int)value / ledCount;
        if (pos <= portion) {
            if (portion <= ledCount / 2) {
                if (portion <= ledCount / 4) {
                    return frc::Color::FromRGB(255, 0, 0);
                }
                return frc::Color::FromRGB(255, 255, 0);
            }
            return frc::Color::FromRGB(0, 255, 0);
        }
        return frc::Color::FromRGB(0, 0, 0);
    }
    void LightBar::lightModeCallback(const std_msgs::msg::Int16 msg) {
        std::cout << "changing lights to mode " << msg.data << std::endl;
        lightMode = static_cast<LightState>(msg.data);
    }
    void LightBar::updateSensorData() {

    }
}
