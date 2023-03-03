// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Subsystems/DoubleArm/DoubleArm.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/SwerveChassis/SwerveChassis.h"


class DropMiddleMove
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
    DropMiddleMove> {
public:
    DropMiddleMove(SwerveChassis* m_swerveChassis);
};
