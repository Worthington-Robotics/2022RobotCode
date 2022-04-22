#pragma once

#include "subsystems/Subsystem.h"
#include "Constants.h"
#include "Util.h"

#include <rclcpp/rclcpp.hpp>
#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>

namespace robot {

    /* Possible control states for the LightStrip to be in */
    enum LightState {
        kTARGETING, /* Shows progress with targeting */
        kINVENTORY, /* Shows balls in possession */
        kALLIANCE, /* Shows alliance color */
        kRAINBOW, /* Turns the robot into a razer mouse */
        kBATTERY, /* Shows battery remaining */
        kTEMPERATURE, /* Shows temperature */
        kTEST, /* Shows the value of a boolean */
        kINDEX, /* Debug that shows light position */
        kONE, /* Debug that turns one led on */
        kSTARS /* Schmoovin */
    };

    class LightBar : public Subsystem {
    public:

        LightBar();

        void createRosBindings(rclcpp::Node *node) override;

        void reset() override;

        void onStart() override;

        void onLoop(double currentTime) override;

        void publishData() override;

        void lightModeCallback(const MSG_INT msg);
    private:

        void execActions();

        /** 
         * Function to get the light color at a position
         * @param pos - Position to get the color at
         **/
        frc::Color getColor(int pos);

        /** 
         * Function to build light modes that display a meter value
         * @param pos - Position to get the color at
         * @param value - Value from 0 to the led count to measure
         **/
        frc::Color meterColor(int pos, int value);

        /* Addressable led bar */
        frc::AddressableLED leds{LED_BAR};
        /* Number of leds */
        static constexpr int ledCount = 40;
        /* Buffer used to hold led color information */
        std::array<frc::AddressableLED::LEDData, ledCount> buffer;

        /* Control states for the lights */
        LightState lightMode = TARGETING;

        void updateSensorData();

        void setAngleOffset(const MSG_FLOAT);
        void setRange(const MSG_FLOAT);

        /* Current time step for modes that animate over time */
        int step = 0;

        /* ROS Subscibers */

        ROS_SUB(MSG_INT) LightModeSub;
        ROS_SUB(MSG_FLOAT) angleOffsetSub;
        ROS_SUB(MSG_FLOAT) rangeSub;

        /* Targeted Bool */
        bool isTargeted = false;
        double angleOffset = 0.0;
        double range = -1;       
    };

} // namespace robot
