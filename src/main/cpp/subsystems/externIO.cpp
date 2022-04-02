#include "subsystems/externIO.h"
#include <frc/smartdashboard/SmartDashboard.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot
{

   ExternIO::ExternIO()
   {
      //internalTOF = std::make_shared<frc::TimeOfFlight>(INTERNAL_TOF_ID);
      //externalTOF = std::make_shared<frc::TimeOfFlight>(EXTERNAL_TOF_ID);
      
   }

   /**
    * Override this function in order to create pulbishers or subscribers against the parent node.
    * NOTE: This function is automatically called by the subsystem manager on registration
    **/
   void ExternIO::createRosBindings(rclcpp::Node *node)
   {


      hoodLimitSwitchResetPub = node->create_publisher<std_msgs::msg::Bool>("/externIO/hood_motor/is_reset", rclcpp::SystemDefaultsQoS());
      for (motors::TalonBrushless * motor : motorsFX)
      {
         motors::MotorContainer MC = {
            *motor,
            node->create_subscription<can_msgs::msg::MotorMsg>("/externIO/" + motor->getName() + "/demand", rclcpp::SystemDefaultsQoS(), std::bind(&motors::Motor::setValue, std::ref(MC.motor), _1)),
            node->create_publisher<sensor_msgs::msg::JointState>("/externIO/" + motor->getName() + "/state", rclcpp::SystemDefaultsQoS()),
            node->create_service<can_msgs::srv::SetPIDFGains>("/externIO/" + motor->getName() + "/pidfset", std::bind(&motors::Motor::configMotorPIDF, std::ref(MC.motor), _1, _2)),
         };

         motorsFXC.push_back(MC);
      }

      for (motors::TalonBrushed * motor : motorsSRX)
      {
         motors::MotorContainer MC = {
            *motor,
            node->create_subscription<can_msgs::msg::MotorMsg>("/externIO/" + motor->getName() + "/demand", rclcpp::SystemDefaultsQoS(), std::bind(&motors::Motor::setValue, std::ref(*motor), _1)),
            node->create_publisher<sensor_msgs::msg::JointState>("/externIO/" + motor->getName() + "/state", rclcpp::SystemDefaultsQoS()),
            node->create_service<can_msgs::srv::SetPIDFGains>("/externIO/" + motor->getName() + "/pidfset", std::bind(&motors::Motor::configMotorPIDF, std::ref(*motor), _1, _2)),
         };
         motorsSRXC.push_back(MC);
      }

      // subscribers of motor demands
      for (solenoid::Solenoid* solenoid : solenoids){
         solenoid::SolenoidContainer SC{
            *solenoid,
            node->create_subscription<std_msgs::msg::Int16>("/externIO/" + solenoid->getName() + "/state", rclcpp::SystemDefaultsQoS(), std::bind(&solenoid::Solenoid::set, std::ref(*solenoid), _1))
         };
         solenoidsC.push_back(SC);
      }
      
      // publishers of sensor data
      externalTOFDistanceSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/external_tof/distance", rclcpp::SensorDataQoS(), std::bind(&ExternIO::setTOF0, this, _1));
      internalTOFDistanceSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/internal_tof/distance", rclcpp::SensorDataQoS(), std::bind(&ExternIO::setTOF1, this, _1));
      upperHoodLimitSwitchPub = node->create_publisher<std_msgs::msg::Bool>("/externIO/upper_hood/limit_switch", rclcpp::SystemDefaultsQoS());
      lowerHoodLimitSwitchPub = node->create_publisher<std_msgs::msg::Bool>("/externIO/lower_hood/limit_switch", rclcpp::SystemDefaultsQoS());
      reset();
   }

   /**
    * Override this function with all the nessecary code needed to reset a subsystem
    **/
   void ExternIO::reset()
   {
      // reset the TOFs
      //externalTOF->SetRangingMode(frc::TimeOfFlight::kShort, 25);
      //internalTOF->SetRangingMode(frc::TimeOfFlight::kShort, 25);

      // setting up the motors
      motorsSRX.at(1)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      motorsSRX.at(1)->getMotor()->ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
      motorsSRX.at(1)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsSRX.at(1)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsSRX.at(1)->getMotor()->SetSensorPhase(false);
      motorsSRX.at(1)->getMotor()->SetInverted(false);
      motorsSRX.at(1)->getMotor()->Config_kP(0, HOOD_KP);
      motorsSRX.at(1)->getMotor()->Config_kI(0, HOOD_KI);
      motorsSRX.at(1)->getMotor()->Config_kD(0, HOOD_KD);
      motorsSRX.at(1)->getMotor()->Config_kF(0, HOOD_KF);
      motorsSRX.at(1)->getMotor()->SetIntegralAccumulator(HOOD_IMAX);
      motorsSRX.at(1)->getMotor()->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
      motorsSRX.at(1)->getMotor()->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
      motorsSRX.at(1)->unmuzzleMotor();

      motorsFX.at(0)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 20, 20, 20});
      motorsFX.at(0)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(0)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(0)->getMotor()->SetSensorPhase(false);
      motorsFX.at(0)->getMotor()->SetInverted(true);
      motorsFX.at(0)->getMotor()->Config_kP(0, INTAKE_KP);
      motorsFX.at(0)->getMotor()->Config_kI(0, INTAKE_KI);
      motorsFX.at(0)->getMotor()->Config_kD(0, INTAKE_KD);
      motorsFX.at(0)->getMotor()->Config_kF(0, INTAKE_KF);
      motorsFX.at(0)->getMotor()->SetIntegralAccumulator(INTAKE_IMAX);
      motorsFX.at(0)->unmuzzleMotor();

      motorsSRX.at(0)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      motorsSRX.at(0)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsSRX.at(0)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsSRX.at(0)->getMotor()->SetSensorPhase(false);
      motorsSRX.at(0)->getMotor()->SetInverted(false);
      motorsSRX.at(0)->getMotor()->Config_kP(0, INDEXER_KP);
      motorsSRX.at(0)->getMotor()->Config_kI(0, INDEXER_KI);
      motorsSRX.at(0)->getMotor()->Config_kD(0, INDEXER_KD);
      motorsSRX.at(0)->getMotor()->Config_kF(0, INDEXER_KF);
      motorsSRX.at(0)->getMotor()->SetIntegralAccumulator(INDEXER_IMAX);
      motorsSRX.at(0)->unmuzzleMotor();
      motorsSRX.at(0)->getMotor()->SetStatusFramePeriod(Status_1_General, 20);

      //shooter

      motorsFX.at(1)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 2});
      motorsFX.at(1)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(1)->getMotor()->SetNeutralMode(motorcontrol::Coast);
      motorsFX.at(1)->getMotor()->SetSensorPhase(true);
      motorsFX.at(1)->getMotor()->SetInverted(true);
      motorsFX.at(1)->getMotor()->Config_kP(0, FLYWHEEL_KP);
      motorsFX.at(1)->getMotor()->Config_kI(0, FLYWHEEL_KI);
      motorsFX.at(1)->getMotor()->Config_kD(0, FLYWHEEL_KD);
      motorsFX.at(1)->getMotor()->Config_kF(0, FLYWHEEL_KF);
      motorsFX.at(1)->getMotor()->SetIntegralAccumulator(FLYWHEEL_IMAX);
      motorsFX.at(1)->unmuzzleMotor();

      motorsFX.at(2)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      motorsFX.at(2)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(2)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(2)->getMotor()->SetSensorPhase(false);
      motorsFX.at(2)->getMotor()->SetInverted(false);
      motorsFX.at(2)->getMotor()->Config_kP(0, CLIMBER_L_KP);
      motorsFX.at(2)->getMotor()->Config_kI(0, CLIMBER_L_KI);
      motorsFX.at(2)->getMotor()->Config_kD(0, CLIMBER_L_KD);
      motorsFX.at(2)->getMotor()->Config_kF(0, CLIMBER_L_KF);
      motorsFX.at(2)->getMotor()->SetIntegralAccumulator(CLIMBER_L_IMAX);
      motorsFX.at(2)->unmuzzleMotor();

      motorsFX.at(3)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      motorsFX.at(3)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(3)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(3)->getMotor()->SetSensorPhase(false);
      motorsFX.at(3)->getMotor()->SetInverted(false);
      motorsFX.at(3)->getMotor()->Config_kP(0, CLIMBER_R_KP);
      motorsFX.at(3)->getMotor()->Config_kI(0, CLIMBER_R_KI);
      motorsFX.at(3)->getMotor()->Config_kD(0, CLIMBER_R_KD);
      motorsFX.at(3)->getMotor()->Config_kF(0, CLIMBER_R_KF);
      motorsFX.at(3)->getMotor()->SetIntegralAccumulator(CLIMBER_R_IMAX);
      motorsFX.at(3)->unmuzzleMotor();

      //this is not the climber -------->

      motorsFX.at(4)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      motorsFX.at(4)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(4)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(4)->getMotor()->SetSensorPhase(false);
      motorsFX.at(4)->getMotor()->SetInverted(false);
      motorsFX.at(4)->getMotor()->Config_kP(0, CLIMBER_R_KP);
      motorsFX.at(4)->getMotor()->Config_kI(0, CLIMBER_R_KI);
      motorsFX.at(4)->getMotor()->Config_kD(0, CLIMBER_R_KD);
      motorsFX.at(4)->getMotor()->Config_kF(0, CLIMBER_R_KF);
      motorsFX.at(4)->getMotor()->SetIntegralAccumulator(CLIMBER_R_IMAX);
      motorsFX.at(4)->unmuzzleMotor();

      for(solenoid::Solenoid* solenoid : solenoids){
         solenoid->getSolenoid()->Set(frc::DoubleSolenoid::Value::kReverse);
      }
   }

   /**
    * Overrride this function with any code needed to be called only once on the first onloop iteration
    **/
   void ExternIO::onStart() {
      
      
   }

   void ExternIO::updateSensorData() {}

   void ExternIO::setTOF0(const std_msgs::msg::Float32 msg){
      frc::SmartDashboard::PutBoolean("externIO/external_tof", (msg.data < .05));
   }

   void ExternIO::setTOF1(const std_msgs::msg::Float32 msg){
      frc::SmartDashboard::PutBoolean("externIO/internal_tof", (msg.data < .05));
   }

   /**
    * Override this function for any code that must be called periodically by the subsystem
    **/
   void ExternIO::onLoop(double currentTime)
   {
      std_msgs::msg::Bool limit;
      if(!motorsSRX.at(1)->getMotor()->IsRevLimitSwitchClosed() && !hoodReset){
         motorsSRX.at(1)->getMotor()->Set(ControlMode::PercentOutput, -.25);
         limit.data = false;
      } else if (motorsSRX.at(1)->getMotor()->IsRevLimitSwitchClosed() && !hoodReset) {
         hoodReset = true;
         motorsSRX.at(1)->getMotor()->SetSelectedSensorPosition(0, 0, 0);
         limit.data = true;
      } else if (motorsSRX.at(1)->getMotor()->IsRevLimitSwitchClosed() || hoodReset) {
         hoodReset = true;
         limit.data = true;
      }
      hoodLimitSwitchResetPub->publish(limit);
      // handling the falcon bits

      for(solenoid::Solenoid* solenoid : solenoids){
         solenoid->getSolenoid()->Set(solenoid->state);
      }

      
   }

   /**
    * Override this function with code needed to publish all data out to the ros network
    **/
   void ExternIO::publishData()
   {
      //std::cout << "publishing data for ExternIO" << std::endl;
      for (motors::MotorContainer MC : motorsFXC)
      {
         // //std::cout << " publishing Talonfx " << std::endl;
         // std::cout << "silenced motor " << std::to_string(MC.shutUp) << std::endl;
         if(!MC.shutUp){

            sensor_msgs::msg::JointState jointState;
            motors::JointState JS = MC.motor.getJointState();
            #ifdef noRosDebug
            frc::SmartDashboard::PutNumber(JS.name + "/position", JS.position);
            frc::SmartDashboard::PutNumber(JS.name + "/velocity", JS.velocity);
            #endif
            jointState.position.push_back(JS.position);
            jointState.velocity.push_back(JS.velocity);
            jointState.effort.push_back(JS.effort);
            jointState.name.push_back(JS.name);
            MC.pub->publish(jointState);
         }
      }

      for (motors::MotorContainer MC : motorsSRXC)
      {
         if(!MC.shutUp){
            sensor_msgs::msg::JointState jointState;
            motors::JointState JS = MC.motor.getJointState();
            jointState.position.push_back(JS.position);
            jointState.velocity.push_back(JS.velocity);
            jointState.effort.push_back(JS.effort);
            jointState.name.push_back(JS.name);
            MC.pub->publish(jointState);
            //std::cout << MC.shutUp << std::endl;
         }
      }      
   }

   void ExternIO::enableDebug(bool debug) {}

}