// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ShotEnum;
import frc.robot.RobotContainer;
import frc.robot.ShotState;

public class AutoShooterShootCommand extends CommandBase {
  Timer shotTimer = new Timer();
  boolean isFinished = false;
  ShotEnum shot;
  ShotState shotState = ShotState.REVERSE;
  double revT = 0.15;
  double spinUpT = .85;
  double shootT = 2;

  /** Creates a new AutoShooterShootCommand. */
  public AutoShooterShootCommand(ShotEnum _shot, double _shootT) {
    addRequirements(RobotContainer.shooter, RobotContainer.intake);
    shot = _shot;
    shootT = _shootT;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShotData.shot = shot;
    shotTimer.reset();
    shotTimer.start();
    RobotContainer.shooter.setAngle(ShotData.getAngle());
    //RobotContainer.shooter.setSpeed(ShotData.getSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterSpeed = 0;
    double intakeSpeed = 0;
    double t = shotTimer.get();
    if(t < revT){
       shotState = ShotState.REVERSE;
    }else if(t < spinUpT + revT) {
      shotState = ShotState.SPINUP;
    }else if(t < shootT + spinUpT + revT){
      shotState = ShotState.SHOOT;
    }  else if(t > shootT + spinUpT + revT) {
      shotState = ShotState.STOP;
    }

switch(shotState){
      case REVERSE:
        intakeSpeed = -1.0;
        shooterSpeed = -0.2;
      break;
      case SPINUP:
        intakeSpeed = 0.0;
        shooterSpeed = ShotData.getSpeed();
      break;
      case SHOOT:
        intakeSpeed = 0.75;
        shooterSpeed = ShotData.getSpeed();
      break;
      case STOP:
        intakeSpeed = 0;
        shooterSpeed = 0;
        isFinished = true;
      break;
    }
    
    RobotContainer.shooter.setSpeed(shooterSpeed);
    RobotContainer.intake.spin(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished)
    {
    RobotContainer.shooter.setSpeed(0);
    RobotContainer.intake.spin(0);
    }
    return isFinished;
  }
}
