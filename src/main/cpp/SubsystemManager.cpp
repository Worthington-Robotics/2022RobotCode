#include "SubsystemManager.h"
#include "units/time.h"
#include <exception>
#include <frc/Errors.h>
#include "rclcpp/rclcpp.hpp"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot {

    SubsystemManager::SubsystemManager() : Node("roborio"),
                                           subsystems(),
                                           enabledNotif(std::bind(&SubsystemManager::enabledLoop, this)),
                                           disabledNotif(std::bind(&SubsystemManager::disabledLoop, this)),
                                           spinNotif(std::bind(&SubsystemManager::spinRos, this)) {
        autoKill = this->create_publisher<MSG_BOOL>("/sys/auto_kill", DEFAULT_QOS);
        headingControlPub = this->create_publisher<MSG_INT>("/drive/heading_control", DEFAULT_QOS);
        shooterRequestPub = this->create_publisher<MSG_INT>("/actions/intake_indexer", DEFAULT_QOS);
        sysReset = this->create_subscription<MSG_BOOL>("/sys/reset", DEFAULT_QOS, std::bind(&SubsystemManager::serviceReset, this, _1));
        sysDebug = this->create_subscription<MSG_BOOL>("/sys/debug", DEFAULT_QOS, std::bind(&SubsystemManager::serviceDebug, this, _1));
        sysEnableEchoPub = this->create_publisher<MSG_INT>("/sys/enable_echo", DEFAULT_QOS);
        spinNotif.StartSingle(0_ms);
    }

    void SubsystemManager::registerSubsystems(std::vector<std::shared_ptr<Subsystem>> subsystems) {
        /* Add all subsystems in new list to the master list */
        this->subsystems.insert(this->subsystems.end(), subsystems.begin(), subsystems.end());

        /* Create the ros bindings for each subsystem and reset its state */
        for (std::shared_ptr<Subsystem> subsystem : subsystems) {
            subsystem->createRosBindings(this);
            subsystem->reset();
            subsystem->enableDebug(false);
        }
    }

    void SubsystemManager::reset() {
        for (std::shared_ptr<Subsystem> subsystem : subsystems) {
            subsystem->reset();
        }
    }

    void SubsystemManager::serviceReset(std::shared_ptr<MSG_BOOL> msg) {
        try {
            reset();
        } catch (std::exception err) {
            std::cout << "There was an issue debugging: " << err.what() << std::endl;
        }
    }

    void SubsystemManager::serviceDebug(std::shared_ptr<MSG_BOOL> msg) {
        try {
            for (std::shared_ptr<Subsystem> subsystem : subsystems) {
                subsystem->enableDebug(msg->data);
            }
        } catch (std::exception err) {
            std::cout << "There was an issue debugging: " << err.what() << std::endl;
        }
    }

    void SubsystemManager::startEnabledLoop() {
        isFirstIteration = true;
        enabledNotif.StartPeriodic(10_ms);

        /**
         * Reset things that could cause issue with the drivers
         * namely the shooting state, the heading control state
         * and the autos running over the endpoint of the phase
         **/
        
        MSG_INT killMsg;
        killMsg.data = 0;
        headingControlPub->publish(killMsg);
        shooterRequestPub->publish(killMsg);
        MSG_BOOL autoEnd;
        autoEnd.data = true;
        autoKill->publish(autoEnd);
    }

    void SubsystemManager::stopEnabledLoop() {
        enabledNotif.Stop();
    }

    void SubsystemManager::startDisabledLoop() {
        disabledNotif.StartPeriodic(10_ms);
        sysDisableTime = GET_TIME_DOUBLE;
    }

    void SubsystemManager::stopDisabledLoop() {
        disabledNotif.Stop();
    }

    void SubsystemManager::spinRos() {
        rclcpp::spin(this->shared_from_this());
    }

    void SubsystemManager::enabledLoop() {
        // double now = GET_TIME_DOUBLE;
        // std::cout << "enabledLoop has began at: " << now << std::endl;
        try {
            if (frc::DriverStation::IsTeleop()) {
                MSG_INT msg;
                msg.data = 1;
                sysEnableEchoPub->publish(msg);
            } else if (frc::DriverStation::IsAutonomousEnabled()) {
                MSG_INT msg;
                msg.data = 2;
                sysEnableEchoPub->publish(msg);
            }
            /* For the first iteration, run onstart */
            if (isFirstIteration) {
                std::cout << "Subsystem first iteration" << std::endl;
                // frc::DriverStation::ReportWarning("Running first iteration");
                for (std::shared_ptr<Subsystem> subsystem : subsystems) {
                    subsystem->onStart();
                    subsystem->updateSensorData();
                    subsystem->publishData();
                }
                isFirstIteration = false;
            /* For all others run onloop */
            } else {
                // frc::DriverStation::ReportWarning("Running onloop iteration");
                for (std::shared_ptr<Subsystem> subsystem : subsystems) {
                    subsystem->updateSensorData();
                    subsystem->onLoop(GET_TIME_DOUBLE);
                    subsystem->publishData();
                }
            }
        } catch (const std::exception &err) {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 131, "enabledLoop()", err.what());
        } catch (...) {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 135, "enabledLoop()", "Looper Thread died with unknown exception");
        }
        // double previousTime = GET_TIME_DOUBLE;
        // dt = now - previousTime;
        // std::cout << "enabledLoop has ended at: " << previousTime << " || dt = " << dt << std::endl;
    }

    void SubsystemManager::disabledLoop() {
        if (GET_TIME_DOUBLE - sysDisableTime > 5) {
            MSG_INT msg;
            msg.data = 0;
            sysEnableEchoPub->publish(msg);
        }
        try {
            // rclcpp::spin_some(this->shared_from_this());
            for (std::shared_ptr<Subsystem> subsystem : subsystems) {
                subsystem->updateSensorData();
                subsystem->publishData();
            }
        } catch (const std::exception &err) {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 131, "disabledLoop()", err.what());
        } catch (...) {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 135, "disabledLoop()", "Looper Thread died with unknown exception");
        }
    }
} // namespace robot