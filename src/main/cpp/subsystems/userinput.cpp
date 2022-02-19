#include "subsystems/userinput.h"
#include <cmath>
#include <string>
#include <frc/Errors.h>

namespace robot
{

    UserInput::UserInput() {}

    void UserInput::registerSticks(std::vector<int> stickIds)
    {
        for (int id : stickIds)
        {
                sticks.push_back(frc::Joystick(id));
        }
    }

    void UserInput::createRosBindings(rclcpp::Node *node)
    {
        for (auto stick = sticks.begin(); stick != sticks.end(); ++stick)
        {
            stickPubs.push_back(
                node->create_publisher<sensor_msgs::msg::Joy>("/sticks/stick" + std::to_string(stick->GetPort()), rclcpp::SensorDataQoS())
            );
        }
    }

    void UserInput::reset() {}

    void UserInput::onStart() {}

    void UserInput::onLoop() {}

    void UserInput::updateSensorData() {}

    void UserInput::publishData()
    {
        for (int i = 0; i < sticks.size(); i++)
        {
            if(frc::DriverStation::IsJoystickConnected(i)){
            sensor_msgs::msg::Joy stickData;

            int numAxes = sticks.at(i).GetAxisCount();
            //std::cout << "Stick " << i << " axis count: " << numAxes << " axis values :[";
            
            std::vector<float> axisValues(numAxes);
            for (int axis = 0; axis < numAxes; axis++)
            {
                axisValues.at(axis) = sticks.at(i).GetRawAxis(axis);
                //std::cout << axisValues.at(axis) << " ";
            }
            stickData.axes = axisValues;
            //std::cout << "]" << std::endl;
             

            int numButtons = sticks.at(i).GetButtonCount();
            std::vector<int> buttonValues(numButtons);
            for(int button = 0; button < numButtons; button++){
                buttonValues.at(button) = sticks.at(i).GetRawButton(button + 1)? 1: 0;
            }
            stickData.buttons = buttonValues;

            stickPubs.at(i)->publish(stickData);

            } else {
                //frc::ReportError(frc::warn::BadJoystickIndex, "userInput.c", 18, "regSticks", "You're about to be real sad because the joystick you want just *Isn't* there :P");
            }
        }
    }

    std::vector<double> UserInput::evalDeadband(const sensor_msgs::msg::Joy &joyMsg,
                                                const double deadBand, const int power)
    {
        auto output = std::vector<double>();
        for (double axis : joyMsg.axes)
        {
            if (std::abs(axis) < deadBand)
            {
                output.push_back(0.0);
            }
            else
            {
                if (axis < 0)
                {
                    output.push_back(-std::abs(std::pow(axis, power)));
                }
                else
                {
                    output.push_back(std::abs(std::pow(axis, power)));
                }
            }
        }
        while(output.size() < 7) {
            output.push_back(0.0);
        }
        return output;
    }

    std::vector<double> UserInput::scalarCut(const sensor_msgs::msg::Joy &joyMsg,
                                             const double deadBand, const int power, const std::vector<double> scalars)
    {
        auto output = evalDeadband(joyMsg, deadBand, power);
        for (int i = 0; i < scalars.size() && i < output.size(); i++)
        {
            output.at(i) = output.at(i) * scalars.at(i);
        }
        return output;
    }

    double UserInput::mapValue(double input, double minOutput, double maxOutput)
    {
        return (input + 1) * (maxOutput - minOutput) / (2) + minOutput;
    }

} // namespace robot
