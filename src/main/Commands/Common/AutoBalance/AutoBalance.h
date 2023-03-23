// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoBalance
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalance> {
public:
    AutoBalance(SwerveChassis* swerveChassis);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    SwerveChassis* m_swerveChassis;
    frc::PIDController xController{ .001, 0, 0 };
    frc::PIDController rController{ .001, 0, 0 };
};
