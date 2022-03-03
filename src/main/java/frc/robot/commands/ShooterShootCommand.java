// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ShotEnum;
import frc.robot.RobotContainer;

public class ShooterShootCommand extends CommandBase {
  Timer shotTimer = new Timer();
  /** Creates a new ShooterShootCommand. */
  public ShooterShootCommand() {
    addRequirements(RobotContainer.shooter, RobotContainer.intake);
  }
  public ShooterShootCommand(ShotEnum _shot){
    this();
    ShotData.shot = _shot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shotTimer.reset();
    shotTimer.start();
    RobotContainer.shooter.setAngle(ShotData.getAngle());
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!shotTimer.hasElapsed(0.15)){
      RobotContainer.intake.spin(-0.7);
    }else {
      RobotContainer.shooter.setSpeed(ShotData.getSpeed());
    }
    if(shotTimer.hasElapsed(1)){
      RobotContainer.intake.spin(0.7);
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
