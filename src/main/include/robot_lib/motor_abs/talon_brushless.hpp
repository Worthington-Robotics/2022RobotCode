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

        void setValue(std::shared_ptr<can_msgs::msg::MotorMsg> msg) override {
            auto controlMode = static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg->control_mode);
            switch(controlMode) {
                case ControlMode::PercentOutput:
                motor->Set(controlMode, msg->demand  * 1024.0 / M_PI);
                break;

                case ControlMode::Velocity:
                motor->Set(controlMode, (msg->demand) / 10.0 * 1024.0 / M_PI);
                break;

                case ControlMode::Position:
                motor->Set(controlMode, msg->demand  * 1024.0 / M_PI);
                break;

                case ControlMode::Follower:
                motor->Set(controlMode, msg->demand);
                break;

                case ControlMode::Disabled:
                default:
                motor->Set(ControlMode::Disabled, 0);
                break;
            }
        }

        JointState getJointState(){
            return {
                name,
                motor->GetSelectedSensorPosition() / 1024.0 * M_PI,
                motor->GetSelectedSensorVelocity() * 10.0 / 1024.0 * M_PI,
                motor->GetStatorCurrent(),
            };
        }

        std::shared_ptr<TalonFX> getMotor(){
            return motor;
        }

        std::string getName(){
            return name + "_motor";
        }

     private:
        std::shared_ptr<TalonFX> motor;
        std::string name;
    };

}  // namespace motors
