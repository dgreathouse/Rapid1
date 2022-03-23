// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMPORTS;


public class IntakeExt extends SubsystemBase {
  /** Creates a new IntakeExt. */
  
  Spark intakeExtMotor = new Spark(PWMPORTS.kIntakeExt);
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  public IntakeExt() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void down(){
    intakeSolenoid.set (Value.kForward);
    intakeExtMotor.set( -1.0);
  }
  public void up() {
    intakeSolenoid.set (Value.kReverse);
    intakeExtMotor.set(0.0);
  }
}
