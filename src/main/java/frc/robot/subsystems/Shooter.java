// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.PWMPORTS;
import frc.robot.commands.ShotData;


public class Shooter extends SubsystemBase {
  private PWMVictorSPX leftMotor = new PWMVictorSPX(PWMPORTS.kLShooter);
  private PWMVictorSPX rightMotor = new PWMVictorSPX(PWMPORTS.kRShooter);
  private TalonSRX angleMotor = new TalonSRX(CANIDS.kShooterAngle);
 

  /** Creates a new Shooter. */
  public Shooter() {
    angleMotor.configOpenloopRamp(.075);
    angleMotor.config_kP(0, 50);
  }
  public void spin(double _speed){
    double spdAdj = RobotContainer.stickOperator.getRawAxis(4);
    spdAdj = (spdAdj + 1)/2;

    double spd = _speed;
    spd = spd + (1-spd) * spdAdj;
    rightMotor.set( spd);
    leftMotor.set( -spd);
  }
  public void setAngle(int _angle){
    //200 is Max
    angleMotor.set(ControlMode.Position, _angle);
  }



  public void setSpeed(double _speed){
    spin(_speed);
  }
  @Override
  public void periodic() {
    SmartDashboard.putString("ShotData", ShotData.shot.toString());
  }
}
