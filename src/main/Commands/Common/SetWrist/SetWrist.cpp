// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetWrist.h"

SetWrist::SetWrist(Intake* intake, bool state): m_Intake(intake), m_state(state) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements(m_Intake);
}

// Called when the command is initially scheduled.
void SetWrist::Initialize() {
    m_Intake->setWristAuto(m_state);
}

// Called repeatedly when this Command is scheduled to run
void SetWrist::Execute() {}

// Called once the command ends or is interrupted.
void SetWrist::End(bool interrupted) {}

// Returns true when the command should end.
bool SetWrist::IsFinished() {
    return true;
}
