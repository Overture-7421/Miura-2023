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
class AutoBalanceRotate
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceRotate> {
public:
    AutoBalanceRotate(SwerveChassis* swerveChassis, double angle);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    SwerveChassis* m_swerveChassis;
    frc::PIDController rController{ .05, 0, 0 };
    double m_Angle;
};
