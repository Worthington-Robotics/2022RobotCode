#include "subsystems/userinput.h"
#include <cmath>
#include <string>
#include <frc/Errors.h>

namespace robot {

    UserInput::UserInput() {}

    void UserInput::registerSticks(std::vector<int> stickIds) {
        for (int id : stickIds) {
            sticks.push_back(frc::Joystick(id));
        }
    }

    void UserInput::createRosBindings(rclcpp::Node *node) {
        intakeIndexerPub = node->create_publisher<std_msgs::msg::Int16>("/actions/intake_indexer", rclcpp::SystemDefaultsQoS());
        intakeSolePub = node->create_publisher<std_msgs::msg::Int16>("/externIO/intake_solenoid/state", rclcpp::SystemDefaultsQoS());
        flyWheelModePub = node->create_publisher<std_msgs::msg::Int16>("/actions/flywheel_mode", rclcpp::SystemDefaultsQoS());
        for (auto stick = sticks.begin(); stick != sticks.end(); ++stick) {
            stickPubs.push_back(
                node->create_publisher<sensor_msgs::msg::Joy>("/sticks/stick" + std::to_string(stick->GetPort()), rclcpp::SensorDataQoS())
            );
        }
    }

    void UserInput::reset() {}

    void UserInput::onStart() {
        std_msgs::msg::Int16 flywheelStart;
        flywheelStart.data = 0;
        flyWheelModePub->publish(flywheelStart);
    }

    void UserInput::onLoop(double currentTime) {}

    void UserInput::updateSensorData() {}

    void UserInput::publishData() {
        for (int i = 0; i < sticks.size(); i++) {
            if (frc::DriverStation::IsJoystickConnected(i)) {
                sensor_msgs::msg::Joy stickData;

                int numAxes = sticks.at(i).GetAxisCount();
                //std::cout << "Stick " << i << " axis count: " << numAxes << " axis values :[";
                
                std::vector<float> axisValues(numAxes);
                for (int axis = 0; axis < numAxes; axis++) {
                    axisValues.at(axis) = sticks.at(i).GetRawAxis(axis);
                    //std::cout << axisValues.at(axis) << " ";
                }
                stickData.axes = axisValues;
                //std::cout << "]" << std::endl;
                

                int numButtons = sticks.at(i).GetButtonCount();
                std::vector<int> buttonValues(numButtons);
                for (int button = 0; button < numButtons; button++) {
                    buttonValues.at(button) = sticks.at(i).GetRawButton(button + 1)? 1: 0;
                }
                stickData.buttons = buttonValues;

                stickPubs.at(i)->publish(stickData);
                if (i == 0) {
                    setStickZero(stickData);
                } else if (i == 1) {
                    setStickOne(stickData);
                }
            } else {
                //frc::ReportError(frc::warn::BadJoystickIndex, "userInput.c", 18, "regSticks", "You're about to be real sad because the joystick you want just *Isn't* there :P");
            }
        }
    }

    std::vector<double> UserInput::evalDeadband(const sensor_msgs::msg::Joy &joyMsg,
                                                const double deadBand, const int power) {
        std::vector<double> output = std::vector<double>();
        for (double axis : joyMsg.axes) {
            if (std::abs(axis) < deadBand) {
                output.push_back(0.0);
            } else {
                if (axis < 0) {
                    output.push_back(-std::abs(std::pow(axis, power)));
                } else {
                    output.push_back(std::abs(std::pow(axis, power)));
                }
            }
        }
        while (output.size() < 7) {
            output.push_back(0.0);
        }
        return output;
    }

    std::vector<double> UserInput::scalarCut(const sensor_msgs::msg::Joy &joyMsg,
                                             const double deadBand, const int power, const std::vector<double> scalars) {
        auto output = evalDeadband(joyMsg, deadBand, power);
        for (int i = 0; i < scalars.size() && i < output.size(); i++) {
            output.at(i) = output.at(i) * scalars.at(i);
        }
        return output;
    }

    double UserInput::mapValue(double input, double minOutput, double maxOutput) {
        return (input + 1) * (maxOutput - minOutput) / (2) + minOutput;
    }

    void UserInput::setStickZero(sensor_msgs::msg::Joy lastStick0) {
        std_msgs::msg::Int16 flywheelModeMsg;
        if(lastStick0.buttons.at(1) && !flywheelModePressed) {
            flywheelModeUpdate = true;
            flywheelModePressed = true;
            flywheelModeMsg.data = 0;
        } else if (lastStick0.buttons.at(2) && !flywheelModePressed) {
            flywheelModeUpdate = true;
            flywheelModePressed = true;
            flywheelModeMsg.data = 2;
        } else if (lastStick0.buttons.at(3) && !flywheelModePressed) {
            flywheelModeUpdate = true;
            flywheelModePressed = true;
            flywheelModeMsg.data = 1;
        } else if (lastStick0.buttons.at(1) || lastStick0.buttons.at(2) || lastStick0.buttons.at(3)) {
            flywheelModePressed = true;
        } else if (flywheelModePressed) {
            flywheelModePressed = false;
        }
        if (flywheelModeUpdate) {
            flyWheelModePub->publish(flywheelModeMsg);
            flywheelModeUpdate = false;
        }
    }

    void UserInput::setStickOne(sensor_msgs::msg::Joy lastStick1) {
        std_msgs::msg::Int16 intakeIndexerMsg;
        if (lastStick1.buttons.at(2) && !intakeIndexerPressed) {
            intakeIndexerMsgUpdate = true;
            intakeIndexerPressed = true;
            intakeIndexerMsg.data = 1;
        } else if (lastStick1.buttons.at(5) && !intakeIndexerPressed) {
            intakeIndexerMsgUpdate = true;
            intakeIndexerPressed = true;
            intakeIndexerMsg.data = -1;
        } else if (lastStick1.buttons.at(0) && !intakeIndexerPressed) {
            intakeIndexerMsgUpdate = true;
            intakeIndexerPressed = true;
            intakeIndexerMsg.data = 2;
        } else if (lastStick1.buttons.at(0) || lastStick1.buttons.at(5) || lastStick1.buttons.at(2)) {
            intakeIndexerPressed = true;
        } else if (intakeIndexerPressed) {
            intakeIndexerPressed = false;
            intakeIndexerMsgUpdate = true;
            intakeIndexerMsg.data = 0;
        }
        if (intakeIndexerMsgUpdate) {
            intakeIndexerPub->publish(intakeIndexerMsg);
            intakeIndexerMsgUpdate = false;
        }

        std_msgs::msg::Int16 intakeSoleMsg;
        if (lastStick1.buttons.at(6) && !intakeSolePressed) {
            intakeSoleMsgUpdate = true;
            intakeSolePressed = true;
            intakeSoleMsg.data = 1;
        } else if (lastStick1.buttons.at(7) && !intakeSolePressed) {
            intakeSoleMsgUpdate = true;
            intakeSolePressed = true;
            intakeSoleMsg.data = -1;
        } else if (lastStick1.buttons.at(6) || lastStick1.buttons.at(7)) {
            intakeSolePressed = true;
        } else if (intakeSolePressed) {
            intakeSolePressed = false;
        }
        if (intakeSoleMsgUpdate) {
            intakeSolePub->publish(intakeSoleMsg);
            intakeSoleMsgUpdate = false;
        }
    }

} // namespace robot
