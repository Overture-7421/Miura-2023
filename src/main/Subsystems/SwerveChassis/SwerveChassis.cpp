// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveChassis.h"

SwerveChassis::SwerveChassis() {
    backRightModule.setRotatorPIDValues(0.09, 0.5, 0, 0);
    backLeftModule.setRotatorPIDValues(0.09, 0.5, 0, 0);
    frontRightModule.setRotatorPIDValues(0.09, 0.5, 0, 0);
    frontLeftModule.setRotatorPIDValues(0.09, 0.5, 0, 0);

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

// Uso para teleoperado
void SwerveChassis::setSpeed(frc::ChassisSpeeds speeds) {
    frontLeftModule.setUseRawVoltageSpeed(false);
    frontRightModule.setUseRawVoltageSpeed(false);
    backLeftModule.setUseRawVoltageSpeed(false);
    backRightModule.setUseRawVoltageSpeed(false);

    this->linearX = speeds.vx.value();
    this->linearY = speeds.vy.value();
    this->angular = speeds.omega.value();

    wpi::array<frc::SwerveModuleState, 4> desiredStates = kinematics.ToSwerveModuleStates(speeds);

    setModuleStates(desiredStates);
}

// Uso para caracterización
void SwerveChassis::setWheelVoltage(double voltage) {
    frontLeftModule.setUseRawVoltageSpeed(true);
    frontRightModule.setUseRawVoltageSpeed(true);
    backLeftModule.setUseRawVoltageSpeed(true);
    backRightModule.setUseRawVoltageSpeed(true);

    frontLeftModule.SetWheelVoltage(voltage);
    frontRightModule.SetWheelVoltage(voltage);
    backLeftModule.SetWheelVoltage(voltage);
    backRightModule.SetWheelVoltage(voltage);
}


frc::Pose2d SwerveChassis::getOdometry() {
    return odometry.GetEstimatedPosition();
}

void SwerveChassis::resetOdometry(frc::Pose2d initPose) {
    odometry.ResetPosition(frc::Rotation2d{ units::degree_t{navx.GetAngle()} }, getModulePosition(), initPose);
}

double SwerveChassis::getHeadingRate() {
    return -navx.GetRate();
}

const frc::SwerveDriveKinematics<4>& SwerveChassis::getKinematics() {
    return kinematics;
}

void SwerveChassis::addVisionMeasurement(frc::Pose2d pose, units::second_t latency) {
    odometry.AddVisionMeasurement(pose, frc::Timer::GetFPGATimestamp() - latency);
}

void SwerveChassis::resetNavx() {
    navx.ZeroYaw();
}

void SwerveChassis::setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    frontLeftModule.setState(desiredStates[0]);
    frontRightModule.setState(desiredStates[1]);
    backRightModule.setState(desiredStates[2]);
    backLeftModule.setState(desiredStates[3]);

    backRightModule.setVoltages();
    backLeftModule.setVoltages();
    frontLeftModule.setVoltages();
    frontRightModule.setVoltages();
}

wpi::array<frc::SwerveModuleState, 4> SwerveChassis::getModuleStates() {
    wpi::array<frc::SwerveModuleState, 4> modulePositions{
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    };
    return modulePositions;
}

wpi::array<frc::SwerveModulePosition, 4> SwerveChassis::getModulePosition() {
    wpi::array<frc::SwerveModulePosition, 4> modulePositions{
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    };
    return modulePositions;
}



// This method will be called once per scheduler run
void SwerveChassis::Periodic() {
    // frc::SmartDashboard::PutNumber("LinearX", linearX);
    // frc::SmartDashboard::PutNumber("LinearY", linearY);
    // frc::SmartDashboard::PutNumber("Angular", angular);

    odometry.Update(frc::Rotation2d(units::degree_t(-navx.GetAngle())), getModulePosition());
    auto estimatedPos = getOdometry();
    frc::SmartDashboard::PutNumber("OdometryX", estimatedPos.X().value());
    frc::SmartDashboard::PutNumber("OdometryY", estimatedPos.Y().value());
    frc::SmartDashboard::PutNumber("AnglenaveX", estimatedPos.Rotation().Degrees().value());


    field2d.SetRobotPose(getOdometry());
}
