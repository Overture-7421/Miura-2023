// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntakeControl.h"

IntakeControl::IntakeControl(Intake* intake, frc::XboxController* joystick): m_intake(intake), m_joystick(joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements(intake);
}

// Called when the command is initially scheduled.
void IntakeControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeControl::Execute() {
    double reverse = Utils::ApplyAxisFilter(-m_joystick->GetLeftTriggerAxis(), 0.3);
    double take = Utils::ApplyAxisFilter(m_joystick->GetRightTriggerAxis(), 0.3);

    if (take > reverse && m_intake->getUltrasonic() > 7.0) { //change for competition (from 7.0 to anothwer value)
        m_intake->setVoltage(4.0);
    } else if (take < reverse) {
        m_intake->setVoltage(-4.0);
    } else {
        m_intake->setVoltage(0);
    }
}

// Called once the command ends or is interrupted.
void IntakeControl::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeControl::IsFinished() {
    return false;
}
