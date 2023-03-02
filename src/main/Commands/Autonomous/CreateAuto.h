// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/DoubleArm/DoubleArm.h"
#include "Subsystems/Intake/Intake.h"


class CreateAuto: public frc2::CommandHelper<frc2::CommandBase, CreateAuto> {
public:
    CreateAuto(SwerveChassis* swerveChassis, DoubleArm* doubleArm, Intake* intake);
    void addCommandsToMap();
    frc2::CommandPtr GenerateAuto();

private:
    SwerveChassis* m_SwerveChassis;
    DoubleArm* m_DoubleArm;
    Intake* m_Intake;

    //Auto
    frc::SendableChooser<std::string> pathChooser;
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
};
