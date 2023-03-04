// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetIntakeSpeed.h"

SetIntakeSpeed::SetIntakeSpeed(Intake* intake, double voltage): m_Intake(intake), m_Voltage(voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements(m_Intake);
}

// Called when the command is initially scheduled.
void SetIntakeSpeed::Initialize() {
    m_Intake->setVoltage(m_Voltage);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeSpeed::Execute() {}

// Called once the command ends or is interrupted.
void SetIntakeSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool SetIntakeSpeed::IsFinished() {
    return true;
}
