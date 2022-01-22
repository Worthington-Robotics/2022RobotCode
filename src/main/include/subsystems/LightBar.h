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
        TARGETING, //shows progress with targeting
        INVENTORY, //shows balls in possession
        ALLIANCE, //shows alliance color
        RAINBOW, //turns the robot into a razer mouse
        BATTERY, //shows battery remaining
        TEMPERATURE, //shows temperature
        TEST, //shows the value of a boolean
        INDEX, //debug that shows light position
        ONE, //debug that turns one led on
        STARS //schmoovin
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

        /** 
         * Function to get the light color at a position
         * @param int pos - Position to get the color at
         **/
        frc::Color getColor(int pos);
        /** 
         * Function to build light modes that display a meter value
         * @param int pos - Position to get the color at
         * @param int value - Value from 0 to the led count to measure
         **/
        frc::Color meterColor(int pos, int value);
        //addressable led bar
        frc::AddressableLED leds{LED_BAR};
        //number of leds
        static constexpr int ledCount = 30;
        //buffer used to hold led color information
        std::array<frc::AddressableLED::LEDData, ledCount> buffer;

        // Control states for the lights
        LightState lightMode = RAINBOW;

        void updateSensorData();

        //current time step for modes that animate over time
        int step = 0;
        int starOffset = 0;

        // ROS Subscibers
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr LightModeSub;

        
    };
} // namespace robot
