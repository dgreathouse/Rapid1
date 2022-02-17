// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ShotEnum;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterDefaultCommand extends CommandBase {
  /** Creates a new ShooterDefaultCommand. */
  public ShooterDefaultCommand(Shooter _shooter) {
    addRequirements(_shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.spin(-0.05);
    if(RobotContainer.stickOperator.getRawButtonPressed(12)){
      ShotData.shot = ShotEnum.BACK_HI;
    }else if(RobotContainer.stickOperator.getRawButtonPressed(11)){
      ShotData.shot = ShotEnum.BACK_LOW;
    }else if(RobotContainer.stickOperator.getRawButtonPressed(8)){
      ShotData.shot = ShotEnum.FRONT_HI_CLOSE;
    }else if(RobotContainer.stickOperator.getRawButtonPressed(9)){
      ShotData.shot = ShotEnum.FRONT_HI_36INCH;
    }else if(RobotContainer.stickOperator.getRawButtonPressed(10)){
      ShotData.shot = ShotEnum.FRONT_HI_LAUNCHPAD;
    }else if(RobotContainer.stickOperator.getRawButtonPressed(7)){
      ShotData.shot = ShotEnum.FRONT_LOW_CLOSE;
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
