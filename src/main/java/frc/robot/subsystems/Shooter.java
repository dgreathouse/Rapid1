// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDS;
import frc.robot.commands.ShotData;

public class Shooter extends SubsystemBase {
  private TalonSRX leftMotor = new TalonSRX(CANIDS.kShooterLeft);
  private TalonSRX rightMotor = new TalonSRX(CANIDS.kShooterRight);
  private TalonSRX angleMotor = new TalonSRX(CANIDS.kShooterAngle);



  /** Creates a new Shooter. */
  public Shooter() {
    
    leftMotor.follow(rightMotor);
    angleMotor.config_kP(0, 150);
    
  }
  public void spin(double _speed){
    rightMotor.set(ControlMode.PercentOutput, _speed);

  }
  public void setAngle(int _angle){
    angleMotor.set(ControlMode.Position, _angle);
  }



  public void setSpeed(double _speed){
    spin(_speed);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Offset", RobotContainer.stickOperator.getRawAxis(3));
    SmartDashboard.putString("Shot Enum", ShotData.shot.toString());
    SmartDashboard.putNumber("Shot Speed", ShotData.getSpeed());
    SmartDashboard.putNumber("Shot Angle", ShotData.getAngle());
    SmartDashboard.putNumber("Shot Current Angle", angleMotor.getSelectedSensorPosition());
SmartDashboard.putNumber("LCurrent", leftMotor.getSupplyCurrent());
SmartDashboard.putNumber("RCurrent", rightMotor.getSupplyCurrent());

  }
}
