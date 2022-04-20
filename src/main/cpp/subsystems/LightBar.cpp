#include "subsystems/LightBar.h"

#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"
#include "Util.h"

using std::placeholders::_1;

namespace robot
{
    LightBar::LightBar() {
        leds.SetLength(60);
    }
    void LightBar::createRosBindings(rclcpp::Node *node) {
        LightModeSub = node->create_subscription<std_msgs::msg::Int16>("/lights/mode", rclcpp::SensorDataQoS(), std::bind(LightBar::lightModeCallback, this, _1));
        angleOffsetSub = node->create_subscription<std_msgs::msg::Float32>("/limelight/angle_offset", rclcpp::SensorDataQoS(), std::bind(LightBar::setAngleOffset, this, _1));
        rangeSub = node->create_subscription<std_msgs::msg::Float32>("/limelight/range", rclcpp::SensorDataQoS(), std::bind(LightBar::setRange, this, _1));
    }
    void LightBar::reset() {
        leds.Start();
    }
    void LightBar::onStart() {}

    void LightBar::onLoop(double currentTime) {}

    void LightBar::publishData() {
        if (range == -1 || angleOffset > LIMELIGHT_MAX_ERROR) {
            isTargeted = false;
        } else {
            isTargeted = true;
        }
        frc::SmartDashboard::PutBoolean("limelight/is_target", isTargeted);
        frc::SmartDashboard::PutNumber("Lights/Mode", lightMode);
        for (int i = 0; i < ledCount; i++) {
            frc::Color color = LightBar::getColor(i);
            buffer[i].SetRGB(color.red * 255, color.green * 255, color.blue * 255);
        }
        step++;
        leds.SetData(buffer);
    }

    frc::Color LightBar::getColor(int pos) {
        double por = (double)pos / (double)ledCount;
        frc::Color output;
        switch (lightMode) {
            case RAINBOW: {
                int speed = 130;
                double scale = ((double)(step % speed) / (double)(speed - 1));
                double h = (por + scale - floor(por + scale)) * 180.0;
                return frc::Color::FromHSV((int)h, 255, 230);
            }
            case TEST: {
                if (GET_TIME_INT % 3 == 0) {
                    return frc::Color(0, 1, 0);
                }
                return frc::Color(1, 0, 0);
            }
            case ALLIANCE: {
                frc::Color allianceColor;
                std::vector<frc::Color> colors = {frc::Color(1, 0, 0), frc::Color(0, 0, 1), frc::Color(0, 0, 0)};
                frc::SmartDashboard::PutNumber("Test/Alliance", frc::DriverStation::GetAlliance());
                allianceColor = colors.at(frc::DriverStation::GetAlliance());
                return allianceColor;
                if (ledCount < 17) {
                    return allianceColor;
                }
                bool pattern[17] = {true, true, true, true, false, true, false, true, true, true, true, false, true, true, true, true, true};
                double margin = (ledCount - 17) / 2;
                frc::SmartDashboard::PutNumber("Test/Margin", margin);
                frc::SmartDashboard::PutNumber("Test/Pos", pos);
                if (pos < margin || pos > margin + 17) {
                    return allianceColor;
                }
                else
                {
                    if (pattern[(int)(pos - margin)]) {
                        std::cout << (int)(pos - margin) << std::endl;
                        return frc::Color(1, 1, 1);
                    }
                }
                return allianceColor;
            }
            case INDEX: {
                return frc::Color::FromHSV((int)(por * 180.0), 255, 255);
            }
            case TEMPERATURE: {
                return LightBar::meterColor(pos, GET_TIME_INT % ledCount);
            }
            case ONE: {
                if (pos == 0) {
                    return frc::Color(1, 1, 1);
                }
                else
                {
                    return frc::Color(0, 0, 0);
                }
            }
            case TARGETING: {
                if (range == -1) {
                    output.red = 1;
                    output.green = 0;
                    output.blue = 0;
                }
                else if (angleOffset > LIMELIGHT_MAX_ERROR) {
                    output.red = 1;
                    output.green = .5;
                    output.blue = 0;
                }
                else
                {
                    output.red = 0;
                    output.green = 1;
                    output.blue = 0;
                }
            }
        }
        double multiplier = 1.0;
        output.red *= multiplier;
        output.green *= multiplier;
        output.blue *= multiplier;
        return output;
    }
    frc::Color LightBar::meterColor(int pos, int value) {
        if (pos <= value) {
            if (value <= ledCount / 2) {
                if (value <= ledCount / 6) {
                    return frc::Color(1, 0, 0);
                }
                return frc::Color(1, 1, 0);
            }
            return frc::Color(0, 1, 0);
        }
        return frc::Color(0, 0, 0);
    }
    void LightBar::lightModeCallback(const std_msgs::msg::Int16 msg) {
        std::cout << "Changing lights to mode " << msg.data << std::endl;
        lightMode = static_cast<LightState>(msg.data);
    }

    void LightBar::setAngleOffset(const std_msgs::msg::Float32 msg) {
        angleOffset = std::abs(msg.data) - (6 / range);
    }

    void LightBar::setRange(const std_msgs::msg::Float32 msg) {
        range = msg.data;
    }

    void LightBar::updateSensorData() {}
}
