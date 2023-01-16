// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveCharacterization.h"

void SwerveCharacterizationRobot::RobotInit() {
    logger.UpdateThreadPriority();
    logger.ClearWhenReceived();

}

void SwerveCharacterizationRobot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void SwerveCharacterizationRobot::DisabledInit() {
    logger.SendData();
}

void SwerveCharacterizationRobot::DisabledPeriodic() {}


void SwerveCharacterizationRobot::AutonomousInit() {
    logger.InitLogging();
}

void SwerveCharacterizationRobot::AutonomousPeriodic() {
    auto leftVoltage = logger.GetLeftMotorVoltage();
    auto rightVoltage = logger.GetRightMotorVoltage();
    frc::SmartDashboard::PutNumber("Sysid/AppliedLeftVoltage", leftVoltage.value());
    frc::SmartDashboard::PutNumber("Sysid/AppliedRightVoltage", rightVoltage.value());

    frc::SwerveModuleState desiredState;
    desiredState.angle = 0_deg;

    swerve.setModuleStates({ desiredState,desiredState,desiredState,desiredState });


    const auto moduleStates = swerve.getModuleStates();
    const auto modulePosition = swerve.getModulePosition();

    double leftPosition = (modulePosition[0].distance + modulePosition[2].distance).value() / 2.0;
    double rightPosition = (modulePosition[1].distance + modulePosition[3].distance).value() / 2.0;

    double leftSpeed = (moduleStates[0].speed + moduleStates[2].speed).value() / 2.0;
    double rightSpeed = (moduleStates[1].speed + moduleStates[3].speed).value() / 2.0;

    double heading = swerve.getOdometry().Rotation().Degrees().value();
    double headingRate = swerve.getHeadingRate();

    logger.Log(leftVoltage.value(), rightVoltage.value(), leftPosition, rightPosition, leftSpeed, rightSpeed, heading, headingRate);

    swerve.setWheelVoltage((leftVoltage + rightVoltage).value() / 2.0);
}


void SwerveCharacterizationRobot::TeleopInit() {

}

void SwerveCharacterizationRobot::TeleopPeriodic() {}


void SwerveCharacterizationRobot::TestInit() {
}

void SwerveCharacterizationRobot::TestPeriodic() {}


