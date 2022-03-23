// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.LIFT;



public class Lift extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax (CANIDS.kLiftLeft, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(CANIDS.kLiftRight, MotorType.kBrushless);

  double m_rollOffset = 0;
  boolean m_liftInitiated = false; 
  /** Creates a new Lift. */
  public Lift() {
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setOpenLoopRampRate(0.5);
    rightMotor.setOpenLoopRampRate(0.5);
  }

  /** Lower the climber arms
   * 
   * @param _speed Psitive speed is up and negative is down.
   */

  public void move(double _speed){
    double leftSpeed = _speed;
    double rightSpeed = -_speed;
    double roll = RobotContainer.driveline.getRobotRoll() - m_rollOffset;
    double leftDis = leftMotor.getEncoder().getPosition();
    double rightDis = rightMotor.getEncoder().getPosition();
    if(_speed > 0){ // Roll compensation when lowering the lift hooks
      leftSpeed = leftSpeed - (roll * 0.05);
      rightSpeed = rightSpeed + (roll * 0.05);
    }
    // Distance limits up 
    if(_speed < 0){
      if(leftDis < -LIFT.kLimitUp){ leftSpeed = 0;}
      if(rightDis > LIFT.kLimitUp){ rightSpeed = 0;}
    }
    // Distance limits down
    if(_speed > 0){
      if(leftDis > LIFT.kLimitDown) { leftSpeed = 0;}
      if(rightDis < -LIFT.kLimitDown) { rightSpeed = 0;}
    }
    // 200 limit Left - neg
    leftMotor.set( leftSpeed);
    rightMotor.set( rightSpeed);
  }
  public void resetEncoders(){
    leftMotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }
  public void setRollOffset(double val) {
    m_rollOffset = val;
  }
  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Roll", RobotContainer.driveline.getRobotRoll() - m_rollOffset);
    SmartDashboard.putNumber("Left Lift Dis", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("right Lift Dis", rightMotor.getEncoder().getPosition());
    // SmartDashboard.putBoolean("Left Up Climber Limit", leftUpLimit.get());
    // SmartDashboard.putBoolean("Left Down Climber Limit", leftDownLimit.get());
    // SmartDashboard.putBoolean("Right Up Climber Limit", rightUpLimit.get());
    // SmartDashboard.putBoolean("Right Down Climber Limit", rightDownLimit.get());
  }
}
