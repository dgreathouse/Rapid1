// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoDelayCommand extends CommandBase {
  Timer delayTimer = new Timer();
  double delayTime = 0;
  /** Creates a new AutoDelayCommand. */
  public AutoDelayCommand(double _time) {
    addRequirements(RobotContainer.driveline, RobotContainer.intake);// here to declare subsystem dependencies.
    delayTime = _time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delayTimer.reset();
    delayTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.intake.isCargoInIntake()){
      RobotContainer.intake.spin(1.0);
    }else {
      RobotContainer.intake.spin(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(delayTimer.hasElapsed(delayTime)){
      return true;
    }
    return false;
  }
}
