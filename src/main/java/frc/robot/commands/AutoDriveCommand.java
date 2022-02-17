// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoDriveCommand extends CommandBase {
  double m_xSpeed, m_ySpeed, m_distanceIn;
  Timer timer;
  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(double _xSpeed, double _ySpeed, double _distanceIn) {

    addRequirements(RobotContainer.driveline); 
    m_xSpeed = _xSpeed;
    m_ySpeed = _ySpeed;
    m_distanceIn = _distanceIn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!timer.hasElapsed(0.5)){
      RobotContainer.driveline.autoRotateWheels(m_xSpeed, m_ySpeed);
      RobotContainer.driveline.resetSwerveDriveEncoders();
    }else {
      RobotContainer.driveline.autoDrive(m_xSpeed, m_ySpeed, m_distanceIn);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}