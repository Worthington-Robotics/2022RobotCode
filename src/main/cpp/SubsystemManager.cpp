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
        sysReset = this->create_subscription<std_msgs::msg::Bool>("/sys/reset", rclcpp::SystemDefaultsQoS(), std::bind(&SubsystemManager::serviceReset, this, _1));
        sysDebug = this->create_subscription<std_msgs::msg::Bool>("/sys/debug", rclcpp::SystemDefaultsQoS(), std::bind(&SubsystemManager::serviceDebug, this, _1));
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

    void SubsystemManager::serviceReset(std::shared_ptr<std_msgs::msg::Bool> msg)
    {
        try
        {
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                subsystem->reset();
            }
        }
        catch (std::exception e)
        {
            std::cout << "There was an issue debugging: " << e.what() << std::endl;
        }
    }

    void SubsystemManager::serviceDebug(std::shared_ptr<std_msgs::msg::Bool> msg)
    {
        try
        {
            isDebug = msg->data;
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                subsystem->enableDebug(msg->data);
            }
        }
        catch (std::exception e)
        {
            std::cout << "There was an issue debugging: " << e.what() << std::endl;
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
                    if(isDebug) std::cout << "Grabbing sensor data" << std::endl;
                    subsystem->updateSensorData();
                    if(isDebug) std::cout << "Looping subsystem" << std::endl;
                    subsystem->onStart();
                    if(isDebug) std::cout << "Publishing data to ros" << std::endl;
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
                    if(isDebug) std::cout << "Grabbing sensor data" << std::endl;
                    subsystem->updateSensorData();
                    if(isDebug) std::cout << "Looping subsystem" << std::endl;
                    subsystem->onLoop(frc::Timer::GetFPGATimestamp().to<double>());
                    if(isDebug) std::cout << "Publishing data to ros" << std::endl;
                    subsystem->publishData();
                }
            }

            rclcpp::spin_some(this->shared_from_this());
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
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                if(isDebug) std::cout << "Grabbing sensor data" << std::endl;
                subsystem->updateSensorData();
                if(isDebug) std::cout << "Publishing data to ros" << std::endl;
                subsystem->publishData();
            }
            rclcpp::spin_some(this->shared_from_this());
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

    /*
    std::shared_ptr<SubsystemManager> SubsystemManager::getInstance()
    {
        if(!manager)
        {
            manager = std::make_shared<SubsystemManager>();
        }
        return manager;
    }*/

} // namespace robot