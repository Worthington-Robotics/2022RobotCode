#include "robot_lib/InputManager.h"
#include <cmath>
#include <string>
#include <frc/Errors.h>

namespace robot
{

	InputManager::InputManager() {}

	void InputManager::createRosBindings(rclcpp::Node *node)
	{
		
	}

	void InputManager::reset() {}

	void InputManager::onStart() {}

	void InputManager::onLoop(double currentTime) {}

	void InputManager::updateSensorData() {}

	void InputManager::publishData()
	{
		
	}

	std::vector<double> InputManager::evalDeadband(const sensor_msgs::msg::Joy &joyMsg,
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
		while (output.size() < 7)
		{
			output.push_back(0.0);
		}
		return output;
	}

	std::vector<double> InputManager::scalarCut(const sensor_msgs::msg::Joy &joyMsg,
											 const double deadBand, const int power, const std::vector<double> scalars)
	{
		auto output = evalDeadband(joyMsg, deadBand, power);
		for (int i = 0; i < scalars.size() && i < output.size(); i++)
		{
			output.at(i) = output.at(i) * scalars.at(i);
		}
		return output;
	}

	double InputManager::mapValue(double input, double minOutput, double maxOutput)
	{
		return (input + 1) * (maxOutput - minOutput) / (2) + minOutput;
	}

	double InputManager::modifierInvert(double value) {
		return 1.0 - value;
	}
} // namespace robot
