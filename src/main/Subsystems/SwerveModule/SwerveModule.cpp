// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

SwerveModule::SwerveModule(int rotatorID, int wheelID, int canCoderID, double offSet) : rotator(rotatorID), wheel(wheelID), canCoder(canCoderID){
  canCoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);

  rotator.ConfigFactoryDefault();
  wheel.ConfigFactoryDefault();
  rotatorPID.EnableContinuousInput(-180, 180);
  canCoder.ConfigSensorDirection(false);
  rotator.SetInverted(true);

  wheel.SetNeutralMode(NeutralMode::Brake);
  rotator.SetNeutralMode(NeutralMode::Brake);
  this->offSet = offSet;
}
double SwerveModule::getSpeed() {
    return getMeters(wheel.GetSelectedSensorVelocity() * 10);
}
double SwerveModule::getMeters(double codes) {
    double meters = codes / 2048 / 6.75 * 0.319024;
    return meters;
}
void SwerveModule::SetRotatorVoltage(double rotatorVoltage) {
    rotator.SetVoltage(units::volt_t(rotatorVoltage));
}
void SwerveModule::SetWheelVoltage(double wheelVoltage) {
    wheel.SetVoltage(units::volt_t(wheelVoltage));
}
double SwerveModule::getAngle() {
    return frc::Rotation2d(units::degree_t(canCoder.GetAbsolutePosition())).RotateBy(units::degree_t(offSet)).Degrees().value();
}
double SwerveModule::getPID(double setPoint) {
    return rotatorPID.Calculate(getAngle(), setPoint);
}
void SwerveModule::setAngle(double angle) {
    this->angle = angle;
}
frc::SwerveModuleState SwerveModule::getState() {
    frc::SwerveModuleState state;

    state.angle = units::degree_t(getAngle());
    state.speed = units::meters_per_second_t(getSpeed());

    return state;
}
void SwerveModule::Periodic() {
    SetRotatorVoltage(getPID(angle));
}
void SwerveModule::setPIDvalues(double kP, double kI, double kD, double f) {
    rotatorPID.SetPID(kP, kI, kD);
    this->f = f;
}