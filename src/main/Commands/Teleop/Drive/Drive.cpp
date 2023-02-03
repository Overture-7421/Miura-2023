// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"
#include "Utils/Utils.h"

Drive::Drive(SwerveChassis* swerveChassis, frc::XboxController* controller): m_swerveChassis(swerveChassis), joystick(controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements(m_swerveChassis);
}

// Called when the command is initially scheduled.
void Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {

    units::meters_per_second_t xInput{ Utils::ApplyAxisFilter(-joystick->GetLeftY()) * 5 };
    units::meters_per_second_t yInput{ Utils::ApplyAxisFilter(-joystick->GetLeftX()) * 5 };
    units::radians_per_second_t rInput{ Utils::ApplyAxisFilter(-joystick->GetRightX()) * 9 };

    frc::ChassisSpeeds chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xLimiter.Calculate(xInput),
        yLimiter.Calculate(yInput),
        rLimiter.Calculate(rInput),
        m_swerveChassis->getOdometry().Rotation());


    m_swerveChassis->setSpeed(chassisSpeeds);
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool Drive::IsFinished() {
    return false;
}
