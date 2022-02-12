#pragma once

#include "robot_lib/motor_abs/motor.hpp"
#include <ctre/Phoenix.h>

namespace motors {
    class TalonBrushless : public Motor {
        public:
        TalonBrushless(int id, std::string motorName) {
            motor = std::make_shared<TalonFX>(id);
            name = motorName;
        }

        void configMotorPIDF(const std::shared_ptr<can_msgs::srv::SetPIDFGains::Request> req,
                             std::shared_ptr<can_msgs::srv::SetPIDFGains::Response> resp) override {
            motor->SelectProfileSlot(req->pid_slot, 0);
            motor->Config_kP(req->pid_slot, req->k_p, 0);
            motor->Config_kI(req->pid_slot, req->k_i, 0);
            motor->Config_IntegralZone(req->pid_slot, req->i_max, 0);
            motor->Config_kD(req->pid_slot, req->k_d, 0);
            motor->Config_kF(req->pid_slot, req->k_f, 0);
            resp->success = motor->GetLastError() == OK;
        }

        void declareConfig(std::shared_ptr<rclcpp::Node> node) override {
            RCLCPP_DEBUG_STREAM(node->get_logger(), "Declaring motor params for " << name);

            // Basic params
            node->declare_parameter<bool>(name + ".inverted", false);
            node->declare_parameter<bool>(name + ".brake_mode", false);
            node->declare_parameter<int>(name + ".follower", -1);
            node->declare_parameter<double>(name + ".vcomp", -1);

            // PID params
            node->declare_parameter<double>(name + ".pid.kp", 0.0);
            node->declare_parameter<double>(name + ".pid.ki", 0.0);
            node->declare_parameter<double>(name + ".pid.kd", 0.0);
            node->declare_parameter<double>(name + ".pid.kf", 0.0);
            node->declare_parameter<double>(name + ".pid.izone", 0.0);

            // Current limit params
            node->declare_parameter<bool>(name + ".current_lim.enable", -1);
            node->declare_parameter<double>(name + ".current_lim.abs_lim", -1);
            node->declare_parameter<double>(name + ".current_lim.lim_trigger", -1);
            node->declare_parameter<double>(name + ".current_lim.time_window", -1);
        }

        void executeConfig(std::shared_ptr<rclcpp::Node> node) override {
            RCLCPP_DEBUG_STREAM(node->get_logger(), "Configuring motor params for " << name);

            // Set basic settings
            motor->ConfigFactoryDefault();
            motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
            motor->SelectProfileSlot(0, 0);
            motor->SetInverted(node->get_parameter(name + ".inverted").as_bool());

            // Configure follower mode (assumes other device is another falcon)
            auto follower = node->get_parameter(name + ".follower").as_int();
            if (follower >= 0)
                motor->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, follower);

            // Configure the neutral mode
            motor->SetNeutralMode(node->get_parameter(name + ".brake_mode").as_bool()
                                      ? NeutralMode::Brake
                                      : NeutralMode::Coast);

            // Configure voltage compensation
            auto vcomp = node->get_parameter(name + ".vcomp").as_double();
            if (vcomp > 0.0) {
                motor->ConfigVoltageCompSaturation(vcomp);
                motor->EnableVoltageCompensation(true);
            }

            // Configure PID settings
            motor->Config_kP(0, node->get_parameter(name + ".pid.kp").as_double(), 0);
            motor->Config_kI(0, node->get_parameter(name + ".pid.ki").as_double(), 0);
            motor->Config_kD(0, node->get_parameter(name + ".pid.kd").as_double(), 0);
            motor->Config_kF(0, node->get_parameter(name + ".pid.kf").as_double(), 0);
            motor->Config_IntegralZone(0, node->get_parameter(name + ".pid.izone").as_double(), 0);

            // Configure Current Limiting
            auto currentLimEnable = node->get_parameter(name + ".current_lim.enable").as_bool();
            auto currentLimitVal = node->get_parameter(name + ".current_lim.abs_lim").as_double();
            auto currentLimitTrigger = node->get_parameter(name + ".current_lim.lim_trigger").as_double();
            auto currentLimitTime = node->get_parameter(name + ".current_lim.time_window").as_double();
            motor->ConfigStatorCurrentLimit({currentLimEnable, currentLimitVal,
                                             currentLimitTrigger,
                                             currentLimitTime});
        }

        void setValue(std::shared_ptr<can_msgs::msg::MotorMsg> msg) override {
            motor->Set(static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg->control_mode), msg->demand);
        }

        JointState getJointState(){
            return {
                name,
                motor->GetSelectedSensorPosition() / 1024.0 * M_PI,
                motor->GetSelectedSensorVelocity() * 10.0 / 1024.0 * M_PI,
                motor->GetStatorCurrent(),
            };
        }

     private:
        std::shared_ptr<TalonFX> motor;
        std::string name;
    };

}  // namespace motors
