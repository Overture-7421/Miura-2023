// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>


class Intake : public frc2::SubsystemBase {
 public:
  Intake();
//Preparado para cuando migremos el código a Intake.cpp
//void setVoltage(double voltage);
//void setPistons(bool state);

 void setVoltage (double voltage) {
  intakeVoltage = units::volt_t(voltage);
  rightIntakeSlave.SetInverted(true);

  leftIntakeMaster.SetVoltage(intakeVoltage);
  rightIntakeSlave.Follow(leftIntakeMaster);
 }

 void setPiston	(bool state) {
    if (state) {
        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
 } 

  void Periodic() override;

 private:
 frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 0, 1};
 WPI_TalonFX leftIntakeMaster {13};
 WPI_TalonFX rightIntakeSlave {14};  
 units::volt_t intakeVoltage;
 
};



//USEFUL DOCUMENTATION (Forgive us, no brain, just ctrl + c, ctrl + v   :D )


/* alabada sea la documentacion
CTRE https://robotpy.readthedocs.io/projects/ctre/en/stable/ctre/WPI_TalonFX.html#ctre.WPI_TalonFX
WPILib https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_p_w_m_talon_f_x.html
Solenoids https://docs.wpilib.org/es/stable/docs/software/hardware-apis/pneumatics/pneumatics.html
*/
