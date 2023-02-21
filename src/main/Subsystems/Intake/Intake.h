// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/AnalogInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Intake: public frc2::SubsystemBase {
public:
    Intake();
    void setVoltage(double voltage);
    void setConeControl();
    void setConeAuto(bool state);
    void setWristControl();
    void setWristAuto(bool state);
    void Periodic() override;

private:
    frc::DoubleSolenoid conePiston{ frc::PneumaticsModuleType::CTREPCM, 2, 0 };
    frc::DoubleSolenoid wristPiston{ frc::PneumaticsModuleType::CTREPCM, 3, 1 };
    WPI_TalonFX intakeMotor{ 15 };
};
