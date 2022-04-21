#include "subsystems/externIO.h"
#include <frc/smartdashboard/SmartDashboard.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot {

   ExternIO::ExternIO() {
      //internalTOF = std::make_shared<frc::TimeOfFlight>(INTERNAL_TOF_ID);
      //externalTOF = std::make_shared<frc::TimeOfFlight>(EXTERNAL_TOF_ID);  
   }

   void ExternIO::createRosBindings(rclcpp::Node *node) {
      hoodLimitSwitchResetPub = node->create_publisher<MSG_BOOL>("/externIO/hood_motor/is_reset", DEFAULT_QOS);
      
      for (motors::TalonBrushless* motor : motorsFX) {
         motors::MotorContainer MC = {
            *motor,
            node->create_subscription<can_msgs::msg::MotorMsg>("/externIO/" + motor->getName() + "/demand", DEFAULT_QOS, std::bind(&motors::Motor::setValue, std::ref(MC.motor), _1)),
            node->create_publisher<sensor_msgs::msg::JointState>("/externIO/" + motor->getName() + "/state", DEFAULT_QOS),
            node->create_service<can_msgs::srv::SetPIDFGains>("/externIO/" + motor->getName() + "/pidfset", std::bind(&motors::Motor::configMotorPIDF, std::ref(MC.motor), _1, _2)),
         };
         motorsFXC.push_back(MC);
      }

      for (motors::TalonBrushed* motor : motorsSRX) {
         motors::MotorContainer MC = {
            *motor,
            node->create_subscription<can_msgs::msg::MotorMsg>("/externIO/" + motor->getName() + "/demand", DEFAULT_QOS, std::bind(&motors::Motor::setValue, std::ref(*motor), _1)),
            node->create_publisher<sensor_msgs::msg::JointState>("/externIO/" + motor->getName() + "/state", DEFAULT_QOS),
            node->create_service<can_msgs::srv::SetPIDFGains>("/externIO/" + motor->getName() + "/pidfset", std::bind(&motors::Motor::configMotorPIDF, std::ref(*motor), _1, _2)),
         };
         motorsSRXC.push_back(MC);
      }

      /* Subscribers of motor demands */
      for (solenoid::Solenoid* solenoid : solenoids) {
         solenoid::SolenoidContainer SC{
            *solenoid,
            node->create_subscription<MSG_INT>("/externIO/" + solenoid->getName() + "/state", DEFAULT_QOS, std::bind(&solenoid::Solenoid::set, std::ref(*solenoid), _1))
         };
         solenoidsC.push_back(SC);
      }
      
      /* Publishers of sensor data */
      externalTOFDistanceSub = node->create_subscription<MSG_FLOAT>("/externIO/external_tof/distance", SENSOR_QOS, std::bind(&ExternIO::setTOF0, this, _1));
      internalTOFDistanceSub = node->create_subscription<MSG_FLOAT>("/externIO/internal_tof/distance", SENSOR_QOS, std::bind(&ExternIO::setTOF1, this, _1));
      upperHoodLimitSwitchPub = node->create_publisher<MSG_BOOL>("/externIO/upper_hood/limit_switch", DEFAULT_QOS);
      lowerHoodLimitSwitchPub = node->create_publisher<MSG_BOOL>("/externIO/lower_hood/limit_switch", DEFAULT_QOS);
      reset();
   }

   void ExternIO::reset() {
      // reset the TOFs
      //externalTOF->SetRangingMode(frc::TimeOfFlight::kShort, 25);
      //internalTOF->SetRangingMode(frc::TimeOfFlight::kShort, 25);

      /* Setting up the motors */
      motors::TalonBrushed* srxMotor;
      motors::TalonBrushless* fxMotor;

      srxMotor = motorsSRX.at(1);
      srxMotor->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      srxMotor->getMotor()->ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
      srxMotor->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      srxMotor->getMotor()->SetNeutralMode(motorcontrol::Brake);
      srxMotor->getMotor()->SetSensorPhase(false);
      srxMotor->getMotor()->SetInverted(false);
      srxMotor->getMotor()->Config_kP(0, HOOD_KP);
      srxMotor->getMotor()->Config_kI(0, HOOD_KI);
      srxMotor->getMotor()->Config_kD(0, HOOD_KD);
      srxMotor->getMotor()->Config_kF(0, HOOD_KF);
      srxMotor->getMotor()->SetIntegralAccumulator(HOOD_IMAX);
      srxMotor->getMotor()->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
      srxMotor->getMotor()->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
      srxMotor->unmuzzleMotor();

      fxMotor = motorsFX.at(0);
      fxMotor->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 20, 20, 20});
      fxMotor->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      fxMotor->getMotor()->SetNeutralMode(motorcontrol::Brake);
      fxMotor->getMotor()->SetSensorPhase(false);
      fxMotor->getMotor()->SetInverted(true);
      fxMotor->getMotor()->Config_kP(0, INTAKE_KP);
      fxMotor->getMotor()->Config_kI(0, INTAKE_KI);
      fxMotor->getMotor()->Config_kD(0, INTAKE_KD);
      fxMotor->getMotor()->Config_kF(0, INTAKE_KF);
      fxMotor->getMotor()->SetIntegralAccumulator(INTAKE_IMAX);
      fxMotor->unmuzzleMotor();

      srxMotor = motorsSRX.at(0);
      srxMotor->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 1});
      srxMotor->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      srxMotor->getMotor()->SetNeutralMode(motorcontrol::Brake);
      srxMotor->getMotor()->SetSensorPhase(false);
      srxMotor->getMotor()->SetInverted(false);
      srxMotor->getMotor()->Config_kP(0, INDEXER_KP);
      srxMotor->getMotor()->Config_kI(0, INDEXER_KI);
      srxMotor->getMotor()->Config_kD(0, INDEXER_KD);
      srxMotor->getMotor()->Config_kF(0, INDEXER_KF);
      srxMotor->getMotor()->SetIntegralAccumulator(INDEXER_IMAX);
      srxMotor->unmuzzleMotor();
      srxMotor->getMotor()->SetStatusFramePeriod(Status_1_General, 20);

      /* Shooter */

      fxMotor = motors.at(1);
      fxMotor->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 5, 10, 2});
      fxMotor->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      fxMotor->getMotor()->SetNeutralMode(motorcontrol::Coast);
      fxMotor->getMotor()->SetSensorPhase(true);
      fxMotor->getMotor()->SetInverted(true);
      fxMotor->getMotor()->Config_kP(0, FLYWHEEL_KP);
      fxMotor->getMotor()->Config_kI(0, FLYWHEEL_KI);
      fxMotor->getMotor()->Config_kD(0, FLYWHEEL_KD);
      fxMotor->getMotor()->Config_kF(0, FLYWHEEL_KF);
      fxMotor->getMotor()->SetIntegralAccumulator(FLYWHEEL_IMAX);
      fxMotor->unmuzzleMotor();

      fxMotor = motors.at(2);
      fxMotor->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      fxMotor->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      fxMotor->getMotor()->SetNeutralMode(motorcontrol::Brake);
      fxMotor->getMotor()->SetSensorPhase(false);
      fxMotor->getMotor()->SetInverted(false);
      fxMotor->getMotor()->Config_kP(0, CLIMBER_L_KP);
      fxMotor->getMotor()->Config_kI(0, CLIMBER_L_KI);
      fxMotor->getMotor()->Config_kD(0, CLIMBER_L_KD);
      fxMotor->getMotor()->Config_kF(0, CLIMBER_L_KF);
      fxMotor->getMotor()->SetIntegralAccumulator(CLIMBER_L_IMAX);
      fxMotor->unmuzzleMotor();

      fxMotor = motors.at(3);
      fxMotor->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      fxMotor->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      fxMotor->getMotor()->SetNeutralMode(motorcontrol::Brake);
      fxMotor->getMotor()->SetSensorPhase(false);
      fxMotor->getMotor()->SetInverted(false);
      fxMotor->getMotor()->Config_kP(0, CLIMBER_R_KP);
      fxMotor->getMotor()->Config_kI(0, CLIMBER_R_KI);
      fxMotor->getMotor()->Config_kD(0, CLIMBER_R_KD);
      fxMotor->getMotor()->Config_kF(0, CLIMBER_R_KF);
      fxMotor->getMotor()->SetIntegralAccumulator(CLIMBER_R_IMAX);
      fxMotor->unmuzzleMotor();

      //this is not the climber -------->

      fxMotor = motorsFx.at(4);
      fxMotor->getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, CLIMBER_HOLD_AMPS, CLIMBER_MAX_AMPS, CLIMBER_MAX_TIME});
      fxMotor->getMotor()->ConfigVoltageCompSaturation(VOLTAGE_COMP);
      fxMotor->getMotor()->SetNeutralMode(motorcontrol::Brake);
      fxMotor->getMotor()->SetSensorPhase(false);
      fxMotor->getMotor()->SetInverted(false);
      fxMotor->getMotor()->Config_kP(0, CLIMBER_R_KP);
      fxMotor->getMotor()->Config_kI(0, CLIMBER_R_KI);
      fxMotor->getMotor()->Config_kD(0, CLIMBER_R_KD);
      fxMotor->getMotor()->Config_kF(0, CLIMBER_R_KF);
      fxMotor->getMotor()->SetIntegralAccumulator(CLIMBER_R_IMAX);
      fxMotor->unmuzzleMotor();

      for (solenoid::Solenoid* solenoid : solenoids) {
         solenoid->getSolenoid()->Set(frc::DoubleSolenoid::Value::kReverse);
      }
   }

   void ExternIO::onStart() {}

   void ExternIO::updateSensorData() {}

   void ExternIO::setTOF0(const MSG_FLOAT msg) {
      frc::SmartDashboard::PutBoolean("externIO/external_tof", (msg.data < .05));
   }

   void ExternIO::setTOF1(const MSG_FLOAT msg) {
      frc::SmartDashboard::PutBoolean("externIO/internal_tof", (msg.data < .05));
   }

   void ExternIO::onLoop(double currentTime) {
      MSG_BOOL limit;
      if (!motorsSRX.at(1)->getMotor()->IsRevLimitSwitchClosed() && !hoodReset) {
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
      /* Handling the falcon bits */

      for (solenoid::Solenoid* solenoid : solenoids) {
         solenoid->getSolenoid()->Set(solenoid->state);
      }
   }

   void ExternIO::publishData() {
      //std::cout << "publishing data for ExternIO" << std::endl;
      for (motors::MotorContainer MC : motorsFXC) {
         // std::cout << " publishing Talonfx " << std::endl;
         // std::cout << "silenced motor " << std::to_string(MC.shutUp) << std::endl;
         if (!MC.shutUp) {
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

      for (motors::MotorContainer MC : motorsSRXC) {
         if (!MC.shutUp) {
            sensor_msgs::msg::JointState jointState;
            motors::JointState JS = MC.motor.getJointState();
            jointState.position.push_back(JS.position);
            jointState.velocity.push_back(JS.velocity);
            jointState.effort.push_back(JS.effort);
            jointState.name.push_back(JS.name);
            MC.pub->publish(jointState);
         }
      }      
   }

   void ExternIO::enableDebug(bool debug) {}
} // namespace robot