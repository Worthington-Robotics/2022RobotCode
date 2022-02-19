#pragma once

#include <vector>

#include <frc/Notifier.h>

#include "rclcpp/rclcpp.hpp"
#include "subsystems/Subsystem.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace robot
{

    class SubsystemManager : public rclcpp::Node
    {
        public: 
        SubsystemManager();

        /**
         * Registers a list of subsystems into the manager for ticking and handling publishing data.
         * They should already have been constructed, but not yet registered. Double registration may cause memory leaks.
         **/ 
        void registerSubsystems(std::vector<std::shared_ptr<Subsystem>> subsystems);

        /**
         * Commands all susbsystems to undergo a reset.
         **/ 
        void reset();

        /**
         * Commands all susbsystems to undergo a reset from a ROS service. 
         * 
         * @ensures
         * pong->success contains whether the reset occured without fail 
         * IF !pong->success, THEN pong->message includes the error message from the failing reset
         * 
         * @param ping This is empty and you can ignore it, it is merely a requirement of ROS
         * @param pong This is the response message with error status
         **/ 
        void serviceReset(std::shared_ptr<std_srvs::srv::Trigger::Request> ping, std::shared_ptr<std_srvs::srv::Trigger::Response> pong);

        /**
         * Commands all susbsystems to enable or disable their debug modes
         * 
         * @ensures
         * pong->success contains whether the debug enable occured without fail 
         * IF !pong->success, THEN pong->message includes the error message from the failing reset
         * 
         * @param ping This contains the ros message, and the boolean for debug to be set to
         * @param pong This is the response message with error status
         **/ 
        void serviceDebug(std::shared_ptr<std_srvs::srv::SetBool::Request> ping, std::shared_ptr<std_srvs::srv::SetBool::Response> pong);

        /**
         * starts the subsystem manager thread, and begins updating the subsystems in order. 
         * Each subsystem is attempted to be updated at a fixed rate of 100 hz.
         * Both the enabled and disabled threads should not be running at the same time.
         **/ 
        void startEnabledLoop();

        /**
         * starts the disabled state subsystem manager thread to keep ticking ros while running.
         * Both the enabled and disabled threads should not be running at the same time.
         **/ 
        void startDisabledLoop();

        /**
         * Stops the susbsystem manager thread. The manager enters a stopped state.
         * Both the enabled and disabled threads should not be running at the same time.
         **/ 
        void stopEnabledLoop();

        /**
         * Stops the disabled thread of the subsystem manager. Both the enabled and disabled
         * threads should not be running at the same time.
         **/ 
        void stopDisabledLoop();

    private:
        bool isFirstIteration = false;
        std::vector<std::shared_ptr<Subsystem>> subsystems;
        frc::Notifier enabledNotif, disabledNotif;

        void enabledLoop();
        void disabledLoop();
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sysReset;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sysDebug;
    };

} // namespace robot