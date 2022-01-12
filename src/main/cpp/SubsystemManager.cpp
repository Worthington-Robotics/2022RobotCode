#include "SubsystemManager.h"
#include "units/time.h"
#include <exception>
#include <frc/Errors.h>
#include "rclcpp/rclcpp.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot
{

    SubsystemManager::SubsystemManager() : Node("roborio"),
                                           subsystems(),
                                           enabledNotif(std::bind(&SubsystemManager::enabledLoop, this)),
                                           disabledNotif(std::bind(&SubsystemManager::disabledLoop, this))
    {
        sysReset = this->create_service<std_srvs::srv::Trigger>("/sys/reset", std::bind(&SubsystemManager::serviceReset, this, _1, _2));
        sysDebug = this->create_service<std_srvs::srv::SetBool>("/sys/debug", std::bind(&SubsystemManager::serviceDebug, this, _1, _2));
        //battery = robot::Battery();
    }

    void SubsystemManager::registerSubsystems(std::vector<std::shared_ptr<Subsystem>> subsystems)
    {
        // Add all subsystems in new list to the master list
        this->subsystems.insert(this->subsystems.end(), subsystems.begin(), subsystems.end());

        // Create the ros bindings for each subsystem and reset its state
        for (std::shared_ptr<Subsystem> subsystem : subsystems)
        {
            subsystem->createRosBindings(this);
            subsystem->reset();
            subsystem->enableDebug(false);
        }
    }

    void SubsystemManager::reset()
    {
        for (std::shared_ptr<Subsystem> subsystem : subsystems)
        {
            subsystem->reset();
        }
    }

    void SubsystemManager::serviceReset(std::shared_ptr<std_srvs::srv::Trigger::Request> ping, std::shared_ptr<std_srvs::srv::Trigger::Response> pong)
    {
        pong->success = true;
        try
        {
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                subsystem->reset();
            }
        }
        catch (std::exception e)
        {
            pong->message = e.what();
            pong->success = false;
        }
    }

    void SubsystemManager::serviceDebug(std::shared_ptr<std_srvs::srv::SetBool::Request> ping, std::shared_ptr<std_srvs::srv::SetBool::Response> pong)
    {
        pong->success = true;
        try
        {
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                subsystem->enableDebug(ping->data);
            }
        }
        catch (std::exception e)
        {
            pong->message = e.what();
            pong->success = false;
        }
    }

    void SubsystemManager::startEnabledLoop()
    {
        isFirstIteration = true;
        enabledNotif.StartPeriodic(10_ms);
    }

    void SubsystemManager::stopEnabledLoop()
    {
        enabledNotif.Stop();
    }

    void SubsystemManager::startDisabledLoop()
    {
        disabledNotif.StartPeriodic(10_ms);
    }

    void SubsystemManager::stopDisabledLoop()
    {
        disabledNotif.Stop();
    }

    void SubsystemManager::enabledLoop()
    {
        try
        {
            // For the first iteration, run onstart
            if (isFirstIteration)
            {
                //frc::DriverStation::ReportWarning("Running first iteration");
                for (std::shared_ptr<Subsystem> subsystem : subsystems)
                {
                    subsystem->onStart();
                    subsystem->updateSensorData();
                    subsystem->publishData();
                }
                isFirstIteration = false;
            }
            // for all others run onloop
            else
            {
                //frc::DriverStation::ReportWarning("Running onloop iteration");
                for (std::shared_ptr<Subsystem> subsystem : subsystems)
                {
                    subsystem->onLoop();
                    subsystem->updateSensorData();
                    subsystem->publishData();
                }
            }

            rclcpp::spin_some(this->shared_from_this());
            //battery stuff
            //frc::SmartDashboard::PutNumber("Drive/Pose/Theta", battery.GetPowerUsage());
        }
        catch (const std::exception &e)
        {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 131, "enabledLoop()", e.what());
        }
        catch (...)
        {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 135, "enabledLoop()", "Looper Thread died with unknown exception");
        }
    }

    void SubsystemManager::disabledLoop()
    {
        try{
            rclcpp::spin_some(this->shared_from_this());
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                subsystem->updateSensorData();
                subsystem->publishData();
            }
        }
        catch (const std::exception &e)
        {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 131, "enabledLoop()", e.what());
        }
        catch (...)
        {
            frc::ReportError(frc::err::Error, "SubsystemManager.cpp", 135, "enabledLoop()", "Looper Thread died with unknown exception");
        }
        //battery stuff
        //frc::SmartDashboard::PutNumber("Drive/Pose/Theta", battery.GetPowerUsage());
    }

} // namespace robot