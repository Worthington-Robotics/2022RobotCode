#include "subsystems/LightBar.h"

#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"

using std::placeholders::_1;

namespace robot {
    LightBar::LightBar(): leds{LED_BAR}{
        leds.SetLength(60);
        //leds.SetSyncTime(units::time::microsecond_t(1.25));
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
        frc::SmartDashboard::PutNumber("Lights/Mode", lightMode);
        for (int i = 0; i < ledCount; i++) {
            //std::cout << i << " " << LightBar::getColor(i).red << " " << LightBar::getColor(i).green << " " << LightBar::getColor(i).blue << std::endl;
            buffer[i].SetLED(LightBar::getColor(i));
        }
        step++;
        leds.SetData(buffer);
    }
    frc::Color LightBar::getColor(int pos) {
        double por = (double)pos / (double)ledCount;
        switch (lightMode) {
            case RAINBOW: {
                int speed = 130;
                double scale = ((double)(step % speed) / (double)(speed - 1));
                double h = (por + scale - floor(por + scale)) * 255.0;
                return frc::Color::FromHSV((int)h, 255, 230);
            }
            case TEST: {
                if (frc::Timer::GetFPGATimestamp().to<int>() % 3 == 0) {
                    return frc::Color(0, 255, 0);
                }
                return frc::Color(255, 0, 0);
            }
            case ALLIANCE: {
                frc::Color allianceColor;
                frc::SmartDashboard::PutNumber("Test/Alliance", frc::DriverStation::GetAlliance());
                switch (frc::DriverStation::GetAlliance()) {
                    case frc::DriverStation::kBlue: {
                        allianceColor = frc::Color(0, 0, 255);
                    }
                    case frc::DriverStation::kRed: {
                        allianceColor = frc::Color(255, 0, 0);
                    }
                    case frc::DriverStation::kInvalid: {
                        allianceColor = frc::Color(0, 0, 0);
                    }
                }
                return allianceColor;
                if (ledCount < 17) {return allianceColor;}
                bool pattern [17] = {true, true, true, true, false, true, false, true, true, true, true, false, true, true, true, true, true};
                double margin = (ledCount - 17) / 2;
                frc::SmartDashboard::PutNumber("Test/Margin", margin);
                frc::SmartDashboard::PutNumber("Test/Pos", pos);
                if (pos < margin || pos > margin + 17) {
                    return allianceColor;
                } else {
                    if (pattern[(int)(pos - margin)]) {
                        std::cout << (int)(pos - margin) << std::endl;
                        return frc::Color(255, 255, 255);
                    }
                }
                return allianceColor;
            }
            case INDEX: {
                return frc::Color::FromHSV((int)(por * 255.0), 255, 255);
            }
            case TEMPERATURE: {
                return LightBar::meterColor(pos, frc::Timer::GetFPGATimestamp().to<int>() % ledCount);
            }
        }
        return frc::Color(0, 0, 0);
    }
    frc::Color LightBar::meterColor(int pos, int value) {
        //int portion = (int)value / ledCount;
        if (pos <= value) {
            if (value <= ledCount / 2) {
                if (value <= ledCount / 6) {
                    return frc::Color(255, 0, 0);
                }
                return frc::Color(255, 255, 0);
            }
            return frc::Color(0, 255, 0);
        }
        return frc::Color(0, 0, 0);
    }
    void LightBar::lightModeCallback(const std_msgs::msg::Int16 msg) {
        std::cout << "changing lights to mode " << msg.data << std::endl;
        lightMode = static_cast<LightState>(msg.data);
    }
    void LightBar::updateSensorData() {

    }
}
