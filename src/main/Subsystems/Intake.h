// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>


class Intake : public frc2::SubsystemBase {
 public:
  Intake();
 
 void setVoltage ();
 /*  
    Codigo pasado
    hacerlo para 2 motores e invertir uno

    Intake() { intakeMotor.SetInverted(true); };
    void setMotor(double voltage) {
    leftIntakeMotor.SetVoltage(units::volt_t(voltage));
  }
*/
 void setPiston	(bool state) {
    if (state) {
        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
 } 
    //Aqui va a ir una función "invertTalonFX" para invertir el movimiento
    void SetInverted(TalonFXInvertType invertType);


    void Set(TalonFXControlMode FollowerType, double 1.0, DemandType NeutralMode, double demand1);
   

   //Aqui va ir una función "setTalofnFX" para definir porcentaje 
    void Set(TalonFXControlMode PercentOutput, double 1.0);
  
  void Periodic() override;

 private:
 
 WPI_TalonFX leftIntakeMotor {13};
 WPI_TalonFX rightIntakeMotor  {14};  
 frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 1, 2};
 
};



//USEFUL DOCUMENTATION (Forgive us, no brain, just ctrl + c, ctrl + v   :D )


/* alabada sea la documentacion
CTRE https://robotpy.readthedocs.io/projects/ctre/en/stable/ctre/WPI_TalonFX.html#ctre.WPI_TalonFX
WPILib https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_p_w_m_talon_f_x.html
Solenoids https://docs.wpilib.org/es/stable/docs/software/hardware-apis/pneumatics/pneumatics.html
*/
