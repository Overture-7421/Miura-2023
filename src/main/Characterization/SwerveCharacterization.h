// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc/TimedRobot.h>
#include <Subsystems/SwerveChassis/SwerveChassis.h>
#include <frc2/command/CommandScheduler.h>
#include "sysid/logging/SysIdDrivetrainLogger.h"

class SwerveCharacterizationRobot: public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;

private:
    sysid::SysIdDrivetrainLogger logger;
    SwerveChassis swerve;
};
