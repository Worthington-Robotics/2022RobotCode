// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>
#include "SubsystemManager.h"
#include "subsystems/drivetrain.h"
#include "subsystems/userinput.h"
#include "subsystems/LightBar.h"
#include "subsystems/externIO.h"
#include "subsystems/climber.h"
#include "AutoSelect.h"


class Robot : public frc::TimedRobot {
  
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
  
  static std::shared_ptr<robot::SubsystemManager> getSubManager();

  private:
  std::shared_ptr<robot::SubsystemManager> manager;
  std::shared_ptr<robot::ExternIO> externIO;
  std::shared_ptr<robot::Drivetrain> drive;
  std::shared_ptr<robot::UserInput> sticks;
  std::shared_ptr<robot::LightBar> lightBar;
  std::shared_ptr<robot::Climber> climber;
  std::shared_ptr<AutoSelect> autoSel;
};
