// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ShotEnum;
import frc.robot.RobotContainer;
import frc.robot.ShotState;

public class ShooterShootCommand extends CommandBase {
  Timer shotTimer = new Timer();
  boolean isFinished = false;
  ShotEnum shot;
  ShotState shotState;// = ShotState.REVERSE;
  //double timeOut = 1;
  double revT = 0.15;
  double spinUpT = 0.85; // Total = spinUpT - revT
  double shootT = 2; // Total = shootT = spinUpT - revT
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
    revT = 0.15;
    spinUpT = 0.85;
    shootT = 2;
    RobotContainer.shooter.setAngle(ShotData.getAngle());
    
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
     // isFinished = true;
    break;
  }
  RobotContainer.shooter.setSpeed(shooterSpeed);
  RobotContainer.intake.spin(intakeSpeed);

    // if(!shotTimer.hasElapsed(0.25)){
    //   RobotContainer.intake.spin(-1.0);
    //   RobotContainer.shooter.setSpeed(-0.2);
    // }else {
    //   RobotContainer.shooter.setSpeed(ShotData.getSpeed());
    //   RobotContainer.intake.spin(0);
    // }
    // if(shotTimer.hasElapsed(1)){
    //   RobotContainer.intake.spin(0.75);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
