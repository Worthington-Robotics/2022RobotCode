#include "subsystems/externIO.h"

using std::placeholders::_1;

namespace robot
{

    ExternIO::ExternIO() {
        flywheelMotor = std::make_shared<TalonFX>(FLYWHEEL_MOTOR_ID);
        deliveryMotor = std::make_shared<TalonFX>(DELIVERY_MOTOR_ID);
        intakeMotor = std::make_shared<TalonFX>(INTAKE_MOTOR_ID);
        climberMotorL = std::make_shared<TalonFX>(CLIMBER_L_MOTOR_ID);
        climberMotorR = std::make_shared<TalonFX>(CLIMBER_R_MOTOR_ID);
        climberMotorC = std::make_shared<TalonFX>(CLIMBER_C_MOTOR_ID);
        hoodMotor = std::make_shared<TalonSRX>(HOOD_MOTOR_ID);
        internalTOF = std::make_shared<frc::TimeOfFlight>(INTERNAL_TOF_ID);
        externalTOF = std::make_shared<frc::TimeOfFlight>(EXTERNAL_TOF_ID);
    }

    /**
     * Override this function in order to create pulbishers or subscribers against the parent node.
     * NOTE: This function is automatically called by the subsystem manager on registration
     **/
    void ExternIO::createRosBindings(rclcpp::Node *node) {
        
        // publishers of sensor data
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hoodEncoderPositionPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flywheelEncoderVelocityPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr externalTOFDistancePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr internalTOFDistancePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr upperHoodLimitSwitchPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lowerHoodLimitSwitchPub;

        // subscribers of motor demands
        hoodMotorPositionDemandSub  = node -> create_subscription<std_msgs::msg::Float32>("/externIO/hood_motor_pos_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setHoodMotorPositionDemand, this, _1));
        flywheelMotorVelocityDemandSub = node -> create_subscription<std_msgs::msg::Float32>("/externIO/flywheel_motor_vel_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setFlywheelMotorVelocityDemand, this, _1));
        deliveryMotorDemandSub = node -> create_subscription<std_msgs::msg::Float32>("/externIO/delivery_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setDeliveryMotorDemand, this, _1));
        intakeMotorDemandSub = node -> create_subscription<std_msgs::msg::Float32>("/externIO/intake_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setIntakeMotorDemand, this, _1));
        climberMotorLDemandSub = node -> create_subscription<std_msgs::msg::Float32>("/externIO/climber_l_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberLMotorDemand, this, _1));
        climberMotorCDemandSub = node -> create_subscription<std_msgs::msg::Float32>("/externIO/climber_c_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberCMotorDemand, this, _1));
        climberMotorRDemandSub = node -> create_subscription<std_msgs::msg::Float32>("/externIO/climber_r_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberRMotorDemand, this, _1));
        //goalPub = node->create_publisher<std_msgs::msg::Float32>("/drive/motor_goal",  rclcpp::SystemDefaultsQoS());
       
    }

    /**
     * Override this function with all the nessecary code needed to reset a subsystem
     **/
    void ExternIO::reset() {}

    /**
     * Overrride this function with any code needed to be called only once on the first onloop iteration
     **/
    void ExternIO::onStart() {}

    /**
     * Override this function for any code that must be called periodically by the subsystem
     **/
    void ExternIO::onLoop(double currentTime) {}

    /**
     * Override this function with code needed to publish all data out to the ros network
     **/
    void ExternIO::publishData() {}

    void ExternIO::enableDebug(bool debug) {}

    void ExternIO::setHoodMotorPositionDemand(const std_msgs::msg::Float32 msg) {
       hoodMotorPositionDemand = msg.data;
    }
    void ExternIO::setFlywheelMotorVelocityDemand(const std_msgs::msg::Float32 msg) {
       flywheelMotorVelocityDemand = msg.data;
    }
    void ExternIO::setDeliveryMotorDemand(const std_msgs::msg::Float32 msg) {
       deliveryMotorDemand = msg.data;
    }
    void ExternIO::setIntakeMotorDemand(const std_msgs::msg::Float32 msg) {
       intakeMotorDemand = msg.data;
    }
    void ExternIO::setClimberLMotorDemand(const std_msgs::msg::Float32 msg) {
       climberLMotorDemand = msg.data;
    }
    void ExternIO::setClimberCMotorDemand(const std_msgs::msg::Float32 msg) {
       climberCMotorDemand = msg.data;
    }
    void ExternIO::setClimberRMotorDemand(const std_msgs::msg::Float32 msg) {
       climberRMotorDemand = msg.data;
    }
}