// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/DoubleArm/DoubleArm.h"
#include <frc/geometry/Translation2d.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
typedef frc::TrapezoidProfile<units::meters> PlannerProfile;
class SetArmCoordinate
    : public frc2::CommandHelper<frc2::CommandBase, SetArmCoordinate> {
public:
    SetArmCoordinate(DoubleArm* doubleArm, frc::Translation2d target, PlannerProfile::Constraints constraints);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    DoubleArm* doubleArm;
    frc::Translation2d target;
    PlannerProfile::Constraints constraints;

};
