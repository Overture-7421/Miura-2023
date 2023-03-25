// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoBalanceRotate.h"

AutoBalanceRotate::AutoBalanceRotate(SwerveChassis* swerveChassis, double angle): m_swerveChassis(swerveChassis), m_Angle(angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements({ m_swerveChassis });
}

// Called when the command is initially scheduled.
void AutoBalanceRotate::Initialize() {
    rController.SetTolerance(0);
}

// Called repeatedly when this Command is scheduled to run
void AutoBalanceRotate::Execute() {
    units::radians_per_second_t rOutput{ rController.Calculate(m_swerveChassis->getYaw(), m_Angle) };

    m_swerveChassis->setSpeed(frc::ChassisSpeeds::FromFieldRelativeSpeeds(0_mps, 0_mps, rOutput, m_swerveChassis->getOdometry().Rotation()));
}

// Called once the command ends or is interrupted.
void AutoBalanceRotate::End(bool interrupted) {
    m_swerveChassis->setSpeed(frc::ChassisSpeeds::FromFieldRelativeSpeeds(0_mps, 0_mps, 0_rad_per_s, m_swerveChassis->getOdometry().Rotation()));
}

// Returns true when the command should end.
bool AutoBalanceRotate::IsFinished() {
    return rController.AtSetpoint();
}
