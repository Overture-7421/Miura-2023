// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int rotatorID, int wheelID, int canCoderID, double offSet, std::string name): rotator(rotatorID), wheel(wheelID), canCoder(canCoderID), name(name) {
    canCoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);

    rotator.ConfigFactoryDefault();
    wheel.ConfigFactoryDefault();
    rotatorPID.EnableContinuousInput(-180, 180);
    canCoder.ConfigSensorDirection(false);
    rotator.SetInverted(true);

    wheel.SetNeutralMode(NeutralMode::Brake);
    rotator.SetNeutralMode(NeutralMode::Coast);
    wheel.SetSelectedSensorPosition(0);
    this->offSet = offSet;
}

double SwerveModule::getSpeed() {
    return getMeters(wheel.GetSelectedSensorVelocity() * 10);
}

double SwerveModule::getDistance() {
    return getMeters(wheel.GetSelectedSensorPosition());
}

double SwerveModule::getMeters(double codes) {
    double meters = codes / 2048 / 6.75 * 0.319024;
    return meters;
}

void SwerveModule::SetRotatorVoltage(double rotatorVoltage) {
    rotator.SetVoltage(units::volt_t(rotatorVoltage));
}

void SwerveModule::SetWheelVoltage(double wheelVoltage) {
    this->wheelVoltage = wheelVoltage;
}

double SwerveModule::getAngle() {
    return frc::Rotation2d(units::degree_t(canCoder.GetAbsolutePosition())).RotateBy(units::degree_t(offSet)).Degrees().value();
}

double SwerveModule::getRotatorPID(double setPoint) {
    return rotatorPID.Calculate(getAngle(), setPoint);
}

double SwerveModule::getWheelPID(double setPoint) {
    return wheelPID.Calculate(getSpeed(), setPoint);
}

frc::SwerveModuleState SwerveModule::getState() {
    frc::SwerveModuleState state;

    state.angle = units::degree_t(getAngle());
    state.speed = units::meters_per_second_t(getSpeed());

    return state;
}

void SwerveModule::setState(frc::SwerveModuleState state) {
    moduleState = frc::SwerveModuleState::Optimize(state, moduleState.angle);
}

frc::SwerveModulePosition SwerveModule::getPosition() {
    return { units::meter_t{getDistance()}, units::degree_t{getAngle()} };
}

void SwerveModule::Periodic() {

    // frc::SmartDashboard::PutNumber(name + "/Distance", getDistance());
    // frc::SmartDashboard::PutNumber(name + "/Angle", getAngle());
    // frc::SmartDashboard::PutNumber(name + "/RawAngle", canCoder.GetAbsolutePosition());
}

void SwerveModule::setRotatorPIDValues(double kP, double kI, double kD, double f) {
    rotatorPID.SetPID(kP, kI, kD);
    this->rotatorF = f;
}

void SwerveModule::setUseRawVoltageSpeed(bool set) {
    useRawVoltageSpeed = set;
}

void SwerveModule::setVoltages() {
    SetRotatorVoltage(getRotatorPID(moduleState.angle.Degrees().value()));

    if (useRawVoltageSpeed) {
        wheel.SetVoltage(units::volt_t(wheelVoltage));
    } else {
        wheel.SetVoltage(units::volt_t{ getWheelPID(moduleState.speed.value()) } + driveFeedForward.Calculate(moduleState.speed));
    }
}
