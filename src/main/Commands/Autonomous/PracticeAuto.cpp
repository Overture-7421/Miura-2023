// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PracticeAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PracticeAuto::PracticeAuto(SwerveChassis* swerveChassis, std::vector<frc::Pose2d> positions, frc::TrajectoryConfig trajectoryConfig): swerveChassis(swerveChassis) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  auto swerveKinematics = swerveChassis->getKinematics();
  trajectoryConfig.SetKinematics(swerveKinematics);

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    positions,
    trajectoryConfig
  );

  frc::ProfiledPIDController<units::radian> thetaController(
    3, 0, 0, { std::numbers::pi * 6_rad_per_s, std::numbers::pi * 6_rad_per_s / 1_s }
  );

  thetaController.EnableContinuousInput(-180_deg, 180_deg);

  frc2::SwerveControllerCommand<4> controllerCommand(
    trajectory,
    [this]() {return this->swerveChassis->getOdometry();},
    swerveChassis->getKinematics(),
    frc::PIDController(0.9, 0, 0),
    frc::PIDController(0.9, 0, 0),
    thetaController,
    [this](auto desiredStates) {this->swerveChassis->setModuleStates(desiredStates);},
    { swerveChassis }
  );

  frc2::InstantCommand stopCommand([this]() {this->swerveChassis->setSpeed(0, 0, 0);});

  AddCommands(controllerCommand, stopCommand);
}
