#pragma once

#include "robot_lib/motor_abs/motor.hpp"
#include <ctre/Phoenix.h>

namespace motors {
    class TalonBrushless : public Motor {
        public:

        TalonBrushless(int id, std::string canBus, std::string motorName) {
            motor = std::make_shared<TalonFX>(id, canBus);
            name = motorName;
        }

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

        void muzzleMotor() {
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 255, 0);
        }

        void unmuzzleMotor() {
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 10, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 255, 0);
            motor->SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 255, 0);
        }

        void setValue(std::shared_ptr<can_msgs::msg::MotorMsg> msg) override {
            // std::cout << "motor " << name << " got data" << std::endl;
            auto controlMode = static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg->control_mode);
            switch(controlMode) {
                case ControlMode::PercentOutput:
                motor->Set(controlMode, msg->demand);
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
