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
  void setVoltage(double voltage);
  void setConeControl();
  void setConeAuto(bool state);
  void setWristControl();
  void setWristAuto(bool state);

  void Periodic() override;

 private:
 frc::DoubleSolenoid coneSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
 frc::DoubleSolenoid wristSolenoid{frc::PneumaticsModuleType::CTREPCM, 2, 3};
 WPI_TalonFX intakeMotor{13};
};


//otro piston




//USEFUL DOCUMENTATION (Forgive us, no brain, just ctrl + c, ctrl + v   :D )


/* alabada sea la documentacion
CTRE https://robotpy.readthedocs.io/projects/ctre/en/stable/ctre/WPI_TalonFX.html#ctre.WPI_TalonFX
WPILib https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_p_w_m_talon_f_x.html
Solenoids https://docs.wpilib.org/es/stable/docs/software/hardware-apis/pneumatics/pneumatics.html
*/
