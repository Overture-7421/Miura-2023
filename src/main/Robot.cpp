// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Characterization/SwerveCharacterization.h"
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
    m_container.setVisionManager();
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_container.setVisionManager();
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
    m_container.setVisionManager();
}

void Robot::TeleopPeriodic() {
    if (m_container.controller.GetRawButton(3)) {
        m_container.doubleArm.SetTargetCoord({ 1.5_m, -0.1_m });
    } else if (m_container.controller.GetRawButton(4)) {
        m_container.doubleArm.SetTargetCoord({ 1.1_m, 1.3_m }); //Altura Máxima
    } else if (m_container.controller.GetRawButton(1)) {
        m_container.doubleArm.SetTargetCoord({ 0.25_m, 0.13_m }); //Posición Inicial
    }
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
