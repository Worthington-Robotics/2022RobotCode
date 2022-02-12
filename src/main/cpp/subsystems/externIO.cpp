#include "subsystems/externIO.h"

using std::placeholders::_1;

namespace robot
{

   ExternIO::ExternIO()
   {
      flywheelMotor = std::make_shared<TalonFX>(FLYWHEEL_MOTOR_ID);
      deliveryMotor = std::make_shared<TalonFX>(DELIVERY_MOTOR_ID);
      intakeMotor = std::make_shared<TalonFX>(INTAKE_MOTOR_ID);
      indexerMotor = std::make_shared<TalonFX>(INDEXER_MOTOR_ID);
      climberMotorL = std::make_shared<TalonFX>(CLIMBER_L_MOTOR_ID);
      climberMotorR = std::make_shared<TalonFX>(CLIMBER_R_MOTOR_ID);
      climberMotorC = std::make_shared<TalonFX>(CLIMBER_C_MOTOR_ID);
      hoodMotor = std::make_shared<TalonSRX>(HOOD_MOTOR_ID);
      internalTOF = std::make_shared<frc::TimeOfFlight>(INTERNAL_TOF_ID);
      externalTOF = std::make_shared<frc::TimeOfFlight>(EXTERNAL_TOF_ID);
      climberSolenoidLR = std::make_shared<frc::DoubleSolenoid>(CLIMBER_SOLENOID_LR_HIGH_ID, CLIMBER_SOLENOID_LR_LOW_ID);
      climberSolenoidC = std::make_shared<frc::DoubleSolenoid>(CLIMBER_SOLENOID_C_HIGH_ID, CLIMBER_SOLENOID_C_LOW_ID);
      intakeSolenoid = std::make_shared<frc::DoubleSolenoid>(INTAKE_SOLENOID_HIGH_ID, INTAKE_SOLENOID_LOW_ID);
      reset();
   }

   /**
    * Override this function in order to create pulbishers or subscribers against the parent node.
    * NOTE: This function is automatically called by the subsystem manager on registration
    **/
   void ExternIO::createRosBindings(rclcpp::Node *node)
   {

      // subscribers of motor demands
      hoodMotorPositionDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/hood_motor_pos_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setHoodMotorPositionDemand, this, _1));
      flywheelMotorVelocityDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/flywheel_motor_vel_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setFlywheelMotorVelocityDemand, this, _1));
      deliveryMotorDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/delivery_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setDeliveryMotorDemand, this, _1));
      intakeMotorDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/intake_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setIntakeMotorDemand, this, _1));
      indexerMotorDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/indexer_motor_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setIndexerMotorDemand, this, _1));
      climberMotorLDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/climber_motor_l_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberMotorLDemand, this, _1));
      climberMotorCDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/climber_motor_c_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberMotorCDemand, this, _1));
      climberMotorRDemandSub = node->create_subscription<std_msgs::msg::Float32>("/externIO/climber_motor_r_demand", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberMotorRDemand, this, _1));
      climberSolenoidLRStateSub = node->create_subscription<std_msgs::msg::Int16>("/externIO/climber_solenoid_lr_state", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberSolenoidLRState, this, _1));
      climberSolenoidCStateSub = node->create_subscription<std_msgs::msg::Int16>("/externIO/climber_solenoid_c_state", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setClimberSolenoidCState, this, _1));
      intakeSolenoidStateSub = node->create_subscription<std_msgs::msg::Int16>("/externIO/intake_solenoid_state", rclcpp::SystemDefaultsQoS(), std::bind(&ExternIO::setIntakeSolenoidState, this, _1));

      // publishers of sensor data
      hoodEncoderPositionPub = node->create_publisher<std_msgs::msg::Float32>("/externIO/hood_encoder_pos", rclcpp::SystemDefaultsQoS());
      flywheelEncoderVelocityPub = node->create_publisher<std_msgs::msg::Float32>("/externIO/flywheel_encoder_vel", rclcpp::SystemDefaultsQoS());
      externalTOFDistancePub = node->create_publisher<std_msgs::msg::Float32>("/externIO/external_tof_distance", rclcpp::SystemDefaultsQoS());
      internalTOFDistancePub = node->create_publisher<std_msgs::msg::Float32>("/externIO/internal_tof_distance", rclcpp::SystemDefaultsQoS());
      upperHoodLimitSwitchPub = node->create_publisher<std_msgs::msg::Bool>("/externIO/upper_hood_limit_switch", rclcpp::SystemDefaultsQoS());
      lowerHoodLimitSwitchPub = node->create_publisher<std_msgs::msg::Bool>("/externIO/lower_hood_limit_switch", rclcpp::SystemDefaultsQoS());
   }

   /**
    * Override this function with all the nessecary code needed to reset a subsystem
    **/
   void ExternIO::reset()
   {
      // reset the TOFs
      externalTOF->SetRangingMode(frc::TimeOfFlight::kShort, 25);
      internalTOF->SetRangingMode(frc::TimeOfFlight::kShort, 25);

      // setting up the motors
      hoodMotor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      hoodMotor->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      hoodMotor->SetNeutralMode(motorcontrol::Brake);
      hoodMotor->SetSensorPhase(false);
      hoodMotor->SetInverted(false);
      hoodMotor->Config_kP(0, HOOD_KP);
      hoodMotor->Config_kI(0, HOOD_KI);
      hoodMotor->Config_kD(0, HOOD_KD);
      hoodMotor->Config_kF(0, HOOD_KF);
      hoodMotor->SetIntegralAccumulator(HOOD_IMAX);
      hoodMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
      hoodMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);

      flywheelMotor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      flywheelMotor->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      flywheelMotor->SetNeutralMode(motorcontrol::Brake);
      flywheelMotor->SetSensorPhase(false);
      flywheelMotor->SetInverted(false);
      flywheelMotor->Config_kP(0, FLYWHEEL_KP);
      flywheelMotor->Config_kI(0, FLYWHEEL_KI);
      flywheelMotor->Config_kD(0, FLYWHEEL_KD);
      flywheelMotor->Config_kF(0, FLYWHEEL_KF);
      flywheelMotor->SetIntegralAccumulator(FLYWHEEL_IMAX);

      indexerMotor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      indexerMotor->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      indexerMotor->SetNeutralMode(motorcontrol::Brake);
      indexerMotor->SetSensorPhase(false);
      indexerMotor->SetInverted(false);
      indexerMotor->Config_kP(0, INDEXER_KP);
      indexerMotor->Config_kI(0, INDEXER_KI);
      indexerMotor->Config_kD(0, INDEXER_KD);
      indexerMotor->Config_kF(0, INDEXER_KF);
      indexerMotor->SetIntegralAccumulator(INDEXER_IMAX);

      intakeMotor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      intakeMotor->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      intakeMotor->SetNeutralMode(motorcontrol::Brake);
      intakeMotor->SetSensorPhase(false);
      intakeMotor->SetInverted(false);
      intakeMotor->Config_kP(0, INTAKE_KP);
      intakeMotor->Config_kI(0, INTAKE_KI);
      intakeMotor->Config_kD(0, INTAKE_KD);
      intakeMotor->Config_kF(0, INTAKE_KF);
      intakeMotor->SetIntegralAccumulator(INTAKE_IMAX);

      climberMotorL->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      climberMotorL->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      climberMotorL->SetNeutralMode(motorcontrol::Brake);
      climberMotorL->SetSensorPhase(false);
      climberMotorL->SetInverted(false);
      climberMotorL->Config_kP(0, CLIMBER_L_KP);
      climberMotorL->Config_kI(0, CLIMBER_L_KI);
      climberMotorL->Config_kD(0, CLIMBER_L_KD);
      climberMotorL->Config_kF(0, CLIMBER_L_KF);
      climberMotorL->SetIntegralAccumulator(CLIMBER_L_IMAX);

      climberMotorC->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      climberMotorC->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      climberMotorC->SetNeutralMode(motorcontrol::Brake);
      climberMotorC->SetSensorPhase(false);
      climberMotorC->SetInverted(false);
      climberMotorC->Config_kP(0, CLIMBER_C_KP);
      climberMotorC->Config_kI(0, CLIMBER_C_KI);
      climberMotorC->Config_kD(0, CLIMBER_C_KD);
      climberMotorC->Config_kF(0, CLIMBER_C_KF);
      climberMotorC->SetIntegralAccumulator(CLIMBER_C_IMAX);

      climberMotorR->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      climberMotorR->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      climberMotorR->SetNeutralMode(motorcontrol::Brake);
      climberMotorR->SetSensorPhase(false);
      climberMotorR->SetInverted(false);
      climberMotorR->Config_kP(0, CLIMBER_R_KP);
      climberMotorR->Config_kI(0, CLIMBER_R_KI);
      climberMotorR->Config_kD(0, CLIMBER_R_KD);
      climberMotorR->Config_kF(0, CLIMBER_R_KF);
      climberMotorR->SetIntegralAccumulator(CLIMBER_R_IMAX);
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
      hoodEncoderPosition.data = hoodMotor->GetSelectedSensorPosition();
      flywheelEncoderVelocity.data = flywheelMotor->GetSelectedSensorVelocity();

      // handling the TOFs
      if (externalTOF->IsRangeValid())
      {
         externalTOFDistance.data = externalTOF->GetRange();
      }
      else
      {
         externalTOFDistance.data = -1;
      }
      if (externalTOF->IsRangeValid())
      {
         internalTOFDistance.data = internalTOF->GetRange();
      }
      else
      {
         internalTOFDistance.data = -1;
      }

      // handling the limit switches
      upperHoodLimitSwitch.data = hoodMotor->IsFwdLimitSwitchClosed();
      lowerHoodLimitSwitch.data = hoodMotor->IsRevLimitSwitchClosed();

      flywheelMotor->Set(ControlMode::Velocity, flywheelMotorVelocityDemand);
      deliveryMotor->Set(ControlMode::PercentOutput, deliveryMotorDemand);
      intakeMotor->Set(ControlMode::PercentOutput, intakeMotorDemand);
      climberMotorL->Set(ControlMode::PercentOutput, climberMotorLDemand);
      climberMotorC->Set(ControlMode::PercentOutput, climberMotorCDemand);
      climberMotorR->Set(ControlMode::PercentOutput, climberMotorRDemand);
      hoodMotor->Set(ControlMode::Position, hoodMotorPositionDemand);

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
      hoodEncoderPositionPub->publish(hoodEncoderPosition);
      flywheelEncoderVelocityPub->publish(flywheelEncoderVelocity);
      externalTOFDistancePub->publish(externalTOFDistance);
      internalTOFDistancePub->publish(internalTOFDistance);
      upperHoodLimitSwitchPub->publish(upperHoodLimitSwitch);
      lowerHoodLimitSwitchPub->publish(lowerHoodLimitSwitch);
   }

   void ExternIO::enableDebug(bool debug) {}

   void ExternIO::setHoodMotorPositionDemand(const std_msgs::msg::Float32 msg)
   {
      hoodMotorPositionDemand = msg.data;
   }
   void ExternIO::setFlywheelMotorVelocityDemand(const std_msgs::msg::Float32 msg)
   {
      flywheelMotorVelocityDemand = msg.data;
   }
   void ExternIO::setDeliveryMotorDemand(const std_msgs::msg::Float32 msg)
   {
      deliveryMotorDemand = msg.data;
   }
   void ExternIO::setIntakeMotorDemand(const std_msgs::msg::Float32 msg)
   {
      intakeMotorDemand = msg.data;
   }
   void ExternIO::setIndexerMotorDemand(const std_msgs::msg::Float32 msg)
   {
      indexerMotorDemand = msg.data;
   }
   void ExternIO::setClimberMotorLDemand(const std_msgs::msg::Float32 msg)
   {
      climberMotorLDemand = msg.data;
   }
   void ExternIO::setClimberMotorCDemand(const std_msgs::msg::Float32 msg)
   {
      climberMotorCDemand = msg.data;
   }
   void ExternIO::setClimberMotorRDemand(const std_msgs::msg::Float32 msg)
   {
      climberMotorRDemand = msg.data;
   }
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