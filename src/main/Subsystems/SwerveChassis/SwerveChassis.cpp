// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveChassis.h"

SwerveChassis::SwerveChassis() {
    backRightModule.setRotatorPIDValues(0.001, 0, 0, 0);
    backLeftModule.setRotatorPIDValues(0.001, 0, 0, 0);
    frontRightModule.setRotatorPIDValues(0.001, 0, 0, 0);
    frontLeftModule.setRotatorPIDValues(0.001, 0, 0, 0);

    backRightModule.setWheelPIDValues(0.001, 0, 0, 0);
    backLeftModule.setWheelPIDValues(0.001, 0, 0, 0);
    frontRightModule.setWheelPIDValues(0.001, 0, 0, 0);
    frontLeftModule.setWheelPIDValues(0.001, 0, 0, 0);

    navx.Calibrate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    double startTime = frc::Timer::GetFPGATimestamp().value();
    while (navx.IsCalibrating()) {
        double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
        if (timePassed > 10) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    navx.ZeroYaw();

    frc::SmartDashboard::PutData("Field", &field2d);
}

void SwerveChassis::setTargetAngle(double targetAngle) {
    this->targetAngle = targetAngle;
}

void SwerveChassis::setSpeed(double linearX, double linearY, double angular) {
    this->linearX = linearX;
    this->linearY = linearY;
    this->angular = angular;

    frc::ChassisSpeeds chassisSpeed;

    frc::SmartDashboard::PutNumber("backLeftModule", backLeftModule.getAngle());
    frc::SmartDashboard::PutNumber("backRightModule", backRightModule.getAngle());
    frc::SmartDashboard::PutNumber("frontLeftModule", frontLeftModule.getAngle());
    frc::SmartDashboard::PutNumber("frontRightModule", frontRightModule.getAngle());

    chassisSpeed.vx = units::meters_per_second_t(linearX);
    chassisSpeed.vy = units::meters_per_second_t(linearY);
    chassisSpeed.omega = units::radians_per_second_t(angular);

    wpi::array<frc::SwerveModuleState, 4> desiredStates = kinematics.ToSwerveModuleStates(chassisSpeed);

    setModuleStates(desiredStates);
}

frc::Pose2d SwerveChassis::getOdometry() {
    return odometry.GetEstimatedPosition();
}

const frc::SwerveDriveKinematics<4>& SwerveChassis::getKinematics() {
    return kinematics;
}

void SwerveChassis::addVisionMeasurement(frc::Pose2d pose, units::second_t latency) {
    odometry.AddVisionMeasurement(pose, frc::Timer::GetFPGATimestamp() - latency);
}

void SwerveChassis::setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    frontLeftModule.setAngle(desiredStates[0].angle.Degrees().value());
    frontRightModule.setAngle(desiredStates[1].angle.Degrees().value());
    backRightModule.setAngle(desiredStates[2].angle.Degrees().value());
    backLeftModule.setAngle(desiredStates[3].angle.Degrees().value());

    frontLeftModule.setSpeed(desiredStates[0].speed.value());
    frontRightModule.setSpeed(desiredStates[1].speed.value());
    backRightModule.setSpeed(desiredStates[2].speed.value());
    backLeftModule.setSpeed(desiredStates[3].speed.value());
}

// This method will be called once per scheduler run
void SwerveChassis::Periodic() {
    frc::SmartDashboard::PutNumber("LinearX", linearX);
    frc::SmartDashboard::PutNumber("LinearY", linearY);
    frc::SmartDashboard::PutNumber("Angular", angular);

    backRightModule.Periodic();
    backLeftModule.Periodic();
    frontLeftModule.Periodic();
    frontRightModule.Periodic();

    wpi::array<frc::SwerveModulePosition, 4> modulePositions{
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    };

    odometry.Update(frc::Rotation2d(units::degree_t(-navx.GetAngle())), modulePositions);

    frc::SmartDashboard::PutNumber("OdometryX", getOdometry().X().value());
    frc::SmartDashboard::PutNumber("OdometryY", getOdometry().Y().value());
    frc::SmartDashboard::PutNumber("AnglenaveX", -navx.GetAngle());


    field2d.SetRobotPose(getOdometry());
}
