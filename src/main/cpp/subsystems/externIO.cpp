#include "subsystems/externIO.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot
{

   ExternIO::ExternIO()
   {
      //internalTOF = std::make_shared<frc::TimeOfFlight>(INTERNAL_TOF_ID);
      //externalTOF = std::make_shared<frc::TimeOfFlight>(EXTERNAL_TOF_ID);
      climberSolenoidLR = std::make_shared<frc::DoubleSolenoid>(frc::PneumaticsModuleType::CTREPCM, CLIMBER_SOLENOID_LR_HIGH_ID, CLIMBER_SOLENOID_LR_LOW_ID);
      climberSolenoidC = std::make_shared<frc::DoubleSolenoid>(frc::PneumaticsModuleType::CTREPCM, CLIMBER_SOLENOID_C_HIGH_ID, CLIMBER_SOLENOID_C_LOW_ID);
      intakeSolenoid = std::make_shared<frc::DoubleSolenoid>(frc::PneumaticsModuleType::CTREPCM, INTAKE_SOLENOID_HIGH_ID, INTAKE_SOLENOID_LOW_ID);
      reset();
   }

   /**
    * Override this function in order to create pulbishers or subscribers against the parent node.
    * NOTE: This function is automatically called by the subsystem manager on registration
    **/
   void ExternIO::createRosBindings(rclcpp::Node *node)
   {

      for (std::shared_ptr<motors::TalonBrushless> motor : motorsFX)
      {
         motors::MotorContainer MC;
         MC.motor = motor;
         MC.pidfSrv = node->create_service<can_msgs::srv::SetPIDFGains>("/externIO/" + motor->getName() + "/pidfset", std::bind(&motors::Motor::configMotorPIDF, std::ref(motor), _1, _2));
         MC.pub = node->create_publisher<sensor_msgs::msg::JointState>("/externIO/" + motor->getName() + "/state", rclcpp::SystemDefaultsQoS());
         MC.sub = node->create_subscription<can_msgs::msg::MotorMsg>("/externIO/" + motor->getName() + "/demand", rclcpp::SystemDefaultsQoS(), std::bind(&motors::Motor::setValue, std::ref(motor), _1));
         motorsFXC.push_back(MC);
      }
      // subscribers of motor demands
      climberSolenoidLRStateSub = node->create_subscription<std_msgs::msg::Int16>("/externIO/climber_solenoid_lr/state", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberSolenoidLRState, this, _1));
      climberSolenoidCStateSub = node->create_subscription<std_msgs::msg::Int16>("/externIO/climber_solenoid_c/state", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberSolenoidCState, this, _1));
      intakeSolenoidStateSub = node->create_subscription<std_msgs::msg::Int16>("/externIO/intake_solenoid/state", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setIntakeSolenoidState, this, _1));

      // publishers of sensor data
      externalTOFDistancePub = node->create_publisher<std_msgs::msg::Float32>("/externIO/external_tof/distance", rclcpp::SystemDefaultsQoS());
      internalTOFDistancePub = node->create_publisher<std_msgs::msg::Float32>("/externIO/internal_tof/distance", rclcpp::SystemDefaultsQoS());
      upperHoodLimitSwitchPub = node->create_publisher<std_msgs::msg::Bool>("/externIO/upper_hood/limit_switch", rclcpp::SystemDefaultsQoS());
      lowerHoodLimitSwitchPub = node->create_publisher<std_msgs::msg::Bool>("/externIO/lower_hood/limit_switch", rclcpp::SystemDefaultsQoS());
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
      motorsSRX.at(0)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      motorsSRX.at(0)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsSRX.at(0)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsSRX.at(0)->getMotor()->SetSensorPhase(false);
      motorsSRX.at(0)->getMotor()->SetInverted(false);
      motorsSRX.at(0)->getMotor()->Config_kP(0, HOOD_KP);
      motorsSRX.at(0)->getMotor()->Config_kI(0, HOOD_KI);
      motorsSRX.at(0)->getMotor()->Config_kD(0, HOOD_KD);
      motorsSRX.at(0)->getMotor()->Config_kF(0, HOOD_KF);
      motorsSRX.at(0)->getMotor()->SetIntegralAccumulator(HOOD_IMAX);
      motorsSRX.at(0)->getMotor()->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
      motorsSRX.at(0)->getMotor()->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);

      motorsFX.at(0)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      motorsFX.at(0)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(0)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(0)->getMotor()->SetSensorPhase(false);
      motorsFX.at(0)->getMotor()->SetInverted(false);
      motorsFX.at(0)->getMotor()->Config_kP(0, INTAKE_KP);
      motorsFX.at(0)->getMotor()->Config_kI(0, INTAKE_KI);
      motorsFX.at(0)->getMotor()->Config_kD(0, INTAKE_KD);
      motorsFX.at(0)->getMotor()->Config_kF(0, INTAKE_KF);
      motorsFX.at(0)->getMotor()->SetIntegralAccumulator(INTAKE_IMAX);

      motorsFX.at(1)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      motorsFX.at(1)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(1)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(1)->getMotor()->SetSensorPhase(false);
      motorsFX.at(1)->getMotor()->SetInverted(false);
      motorsFX.at(1)->getMotor()->Config_kP(0, INDEXER_KP);
      motorsFX.at(1)->getMotor()->Config_kI(0, INDEXER_KI);
      motorsFX.at(1)->getMotor()->Config_kD(0, INDEXER_KD);
      motorsFX.at(1)->getMotor()->Config_kF(0, INDEXER_KF);
      motorsFX.at(1)->getMotor()->SetIntegralAccumulator(INDEXER_IMAX);

      motorsFX.at(2)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      motorsFX.at(2)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(2)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(2)->getMotor()->SetSensorPhase(false);
      motorsFX.at(2)->getMotor()->SetInverted(false);
      motorsFX.at(2)->getMotor()->Config_kP(0, INDEXER_KP);
      motorsFX.at(2)->getMotor()->Config_kI(0, INDEXER_KI);
      motorsFX.at(2)->getMotor()->Config_kD(0, INDEXER_KD);
      motorsFX.at(2)->getMotor()->Config_kF(0, INDEXER_KF);
      motorsFX.at(2)->getMotor()->SetIntegralAccumulator(INDEXER_IMAX);

      motorsFX.at(3)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      motorsFX.at(3)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(3)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(3)->getMotor()->SetSensorPhase(false);
      motorsFX.at(3)->getMotor()->SetInverted(false);
      motorsFX.at(3)->getMotor()->Config_kP(0, FLYWHEEL_KP);
      motorsFX.at(3)->getMotor()->Config_kI(0, FLYWHEEL_KI);
      motorsFX.at(3)->getMotor()->Config_kD(0, FLYWHEEL_KD);
      motorsFX.at(3)->getMotor()->Config_kF(0, FLYWHEEL_KF);
      motorsFX.at(3)->getMotor()->SetIntegralAccumulator(FLYWHEEL_IMAX);

      motorsFX.at(4)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      motorsFX.at(4)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(4)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(4)->getMotor()->SetSensorPhase(false);
      motorsFX.at(4)->getMotor()->SetInverted(false);
      motorsFX.at(4)->getMotor()->Config_kP(0, CLIMBER_L_KP);
      motorsFX.at(4)->getMotor()->Config_kI(0, CLIMBER_L_KI);
      motorsFX.at(4)->getMotor()->Config_kD(0, CLIMBER_L_KD);
      motorsFX.at(4)->getMotor()->Config_kF(0, CLIMBER_L_KF);
      motorsFX.at(4)->getMotor()->SetIntegralAccumulator(CLIMBER_L_IMAX);

      motorsFX.at(5)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      motorsFX.at(5)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(5)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(5)->getMotor()->SetSensorPhase(false);
      motorsFX.at(5)->getMotor()->SetInverted(false);
      motorsFX.at(5)->getMotor()->Config_kP(0, CLIMBER_C_KP);
      motorsFX.at(5)->getMotor()->Config_kI(0, CLIMBER_C_KI);
      motorsFX.at(5)->getMotor()->Config_kD(0, CLIMBER_C_KD);
      motorsFX.at(5)->getMotor()->Config_kF(0, CLIMBER_C_KF);
      motorsFX.at(5)->getMotor()->SetIntegralAccumulator(CLIMBER_C_IMAX);

      motorsFX.at(6)->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      motorsFX.at(6)->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      motorsFX.at(6)->getMotor()->SetNeutralMode(motorcontrol::Brake);
      motorsFX.at(6)->getMotor()->SetSensorPhase(false);
      motorsFX.at(6)->getMotor()->SetInverted(false);
      motorsFX.at(6)->getMotor()->Config_kP(0, CLIMBER_R_KP);
      motorsFX.at(6)->getMotor()->Config_kI(0, CLIMBER_R_KI);
      motorsFX.at(6)->getMotor()->Config_kD(0, CLIMBER_R_KD);
      motorsFX.at(6)->getMotor()->Config_kF(0, CLIMBER_R_KF);
      motorsFX.at(6)->getMotor()->SetIntegralAccumulator(CLIMBER_R_IMAX);
   }

   /**
    * Overrride this function with any code needed to be called only once on the first onloop iteration
    **/
   void ExternIO::onStart() {}

   /**
    * Override this function for any code that must be called periodically by the subsystem
    **/
   void ExternIO::onLoop(double currentTime)
   {
      // handling the falcon bits

      // handling the TOFs
      if (true /*externalTOF->IsRangeValid()*/)
      {
         //externalTOFDistance.data = externalTOF->GetRange();
      }
      else
      {
         externalTOFDistance.data = -1;
      }
      if (true /*externalTOF->IsRangeValid()*/)
      {
         //internalTOFDistance.data = internalTOF->GetRange();
      }
      else
      {
         internalTOFDistance.data = -1;
      }

      // handling the limit switches
      upperHoodLimitSwitch.data = motorsSRX.at(0)->getMotor()->IsFwdLimitSwitchClosed();
      lowerHoodLimitSwitch.data = motorsSRX.at(0)->getMotor()->IsRevLimitSwitchClosed();

      if (climberSolenoidCState < 0)
      {
         climberSolenoidC->Set(frc::DoubleSolenoid::kReverse);
      }
      else if (climberSolenoidCState > 0)
      {
         climberSolenoidC->Set(frc::DoubleSolenoid::kForward);
      }
      else
      {
         climberSolenoidC->Set(frc::DoubleSolenoid::kOff);
      }

      if (climberSolenoidCState < 0)
      {
         climberSolenoidLR->Set(frc::DoubleSolenoid::kReverse);
      }
      else if (climberSolenoidCState > 0)
      {
         climberSolenoidLR->Set(frc::DoubleSolenoid::kForward);
      }
      else
      {
         climberSolenoidLR->Set(frc::DoubleSolenoid::kOff);
      }

      if (intakeSolenoidState < 0)
      {
         intakeSolenoid->Set(frc::DoubleSolenoid::kReverse);
      }
      else if (intakeSolenoidState > 0)
      {
         intakeSolenoid->Set(frc::DoubleSolenoid::kForward);
      }
      else
      {
         intakeSolenoid->Set(frc::DoubleSolenoid::kOff);
      }
   }

   /**
    * Override this function with code needed to publish all data out to the ros network
    **/
   void ExternIO::publishData()
   {
      for (motors::MotorContainer MC : motorsFXC)
      {
         sensor_msgs::msg::JointState jointState;
         motors::JointState JS = MC.motor->getJointState();
         jointState.position.push_back(JS.position);
         jointState.velocity.push_back(JS.velocity);
         jointState.effort.push_back(JS.effort);
         jointState.name.push_back(JS.name);
         MC.pub->publish(jointState);
      }

      for (motors::MotorContainer MC : motorsSRXC)
      {
         sensor_msgs::msg::JointState jointState;
         motors::JointState JS = MC.motor->getJointState();
         jointState.position.push_back(JS.position);
         jointState.velocity.push_back(JS.velocity);
         jointState.effort.push_back(JS.effort);
         jointState.name.push_back(JS.name);
         MC.pub->publish(jointState);
      }

      

      externalTOFDistancePub->publish(externalTOFDistance);
      internalTOFDistancePub->publish(internalTOFDistance);
      upperHoodLimitSwitchPub->publish(upperHoodLimitSwitch);
      lowerHoodLimitSwitchPub->publish(lowerHoodLimitSwitch);
   }

   void ExternIO::enableDebug(bool debug) {}

   void ExternIO::updateSensorData() {}

   void ExternIO::setClimberSolenoidLRState(const std_msgs::msg::Int16 msg)
   {
      climberSolenoidLRState = msg.data;
   }
   void ExternIO::setClimberSolenoidCState(const std_msgs::msg::Int16 msg)
   {
      climberSolenoidCState = msg.data;
   }
   void ExternIO::setIntakeSolenoidState(const std_msgs::msg::Int16 msg)
   {
      intakeSolenoidState = msg.data;
   }
}