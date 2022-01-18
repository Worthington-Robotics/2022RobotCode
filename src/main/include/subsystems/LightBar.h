#pragma once

#include "subsystems/Subsystem.h"
#include <rclcpp/rclcpp.hpp>

#include "frc/AddressableLED.h"
#include "Constants.h"
#include "frc/util/Color.h"
#include "frc/Timer.h"
#include "frc/DriverStation.h"

#include <std_msgs/msg/int16.hpp>

namespace robot
{

    /**
     * Possible control states for the LightStrip to be in
     **/
    enum LightState
    {
        TARGETING, 
        INVENTORY, 
        ALLIANCE, 
        RAINBOW, 
        BATTERY, 
        TEMPERATURE,
        TEST,
        INDEX
    };

    class LightBar : public Subsystem
    {
        public:

        LightBar();

        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/
        void createRosBindings(rclcpp::Node *node) override;

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset() override;

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         **/
        void onStart() override;

        /**
         * Override this function for any code that must be called periodically by the subsystem
         **/
        void onLoop() override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

        /**
         * Callbacks for ROS Subscribers 
         **/

        void lightModeCallback(const std_msgs::msg::Int16 msg);

    private:

        void execActions();

        frc::Color getColor(int pos);
        frc::Color meterColor(int pos, int value);

        static constexpr int ledCount = 30;
        frc::AddressableLED leds;
        std::array<frc::AddressableLED::LEDData, ledCount> buffer;

        // Control states for the lights
        LightState lightMode = TEMPERATURE;
        void updateSensorData();

        int step = 0;
        //IO devices
        //std::shared_ptr<PigeonIMU> imu;

        // ROS Subscibers
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr LightModeSub;

        
    };
} // namespace robot
