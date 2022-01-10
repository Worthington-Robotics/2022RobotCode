#pragma once
#include <rclcpp/rclcpp.hpp>
#include <frc/DriverStation.h>

namespace robot {

    class Subsystem {
        public:
        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/ 
        virtual void createRosBindings(rclcpp::Node * node) = 0;

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/ 
        virtual void reset() = 0;

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         **/ 
        virtual void onStart() = 0;

        /**
         * Override this function for any code that must be called periodically by the subsystem
         **/ 
        virtual void onLoop() = 0;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/ 
        virtual void publishData() = 0;

        /**
         * Override this function with code needed to update sensors assosiated with the robot
         **/ 
        virtual void updateSensorData() = 0;

        /**
         * Overide this function to recieve information on when debug mode is enabled
         **/ 
        virtual void enableDebug(bool debugEnable){
            frc::DriverStation::ReportWarning("At least one subsystem does not override enableDebug");
        }

    };
    
} // namespace robot
