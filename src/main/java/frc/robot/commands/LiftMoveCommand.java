// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LiftMoveCommand extends CommandBase {
  /** Creates a new LiftMoveCommand. */
  public LiftMoveCommand() {
    addRequirements(RobotContainer.lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.driveline.resetGyro();
    //RobotContainer.lift.resetEncoders();
    RobotContainer.lift.setRollOffset(RobotContainer.driveline.getRobotRoll());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.stickOperator.getY();
    RobotContainer.lift.move(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.lift.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
