// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;

public class IntakeExt extends SubsystemBase {
  /** Creates a new IntakeExt. */
  
  VictorSPX intakeExtMotor = new VictorSPX(CANIDS.kIntakeExtMotor);
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  public IntakeExt() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void down(){
    intakeSolenoid.set (Value.kForward);
    intakeExtMotor.set(ControlMode.PercentOutput, 0.7);
  }
  public void up() {
    intakeSolenoid.set (Value.kReverse);
    intakeExtMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
