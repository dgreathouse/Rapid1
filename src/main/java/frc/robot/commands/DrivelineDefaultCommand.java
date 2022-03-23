// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants.OI;
import frc.robot.subsystems.Driveline;

public class DrivelineDefaultCommand extends CommandBase {

  SlewRateLimiter xSRL = new SlewRateLimiter(0.5);
  SlewRateLimiter ySRL = new SlewRateLimiter(0.5);
  SlewRateLimiter twistSRL = new SlewRateLimiter(0.5);

  boolean isFieldOrientedMode = false;

  /** Creates a new DrivelineDefaultCommand. */
  public DrivelineDefaultCommand(Driveline _driveline) {
    addRequirements(_driveline);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Read Joystick values for X, Y and rotation
    double x = RobotContainer.stickDriver.getRawAxis(0);
    double y = -RobotContainer.stickDriver.getRawAxis(1);
    double twist = RobotContainer.stickDriver.getRawAxis(2);

    if(RobotContainer.stickOperator.getRawButton(1)){
      x = RobotContainer.stickOperator.getRawAxis(0);
      y = -RobotContainer.stickOperator.getRawAxis(1);
      twist = RobotContainer.stickOperator.getRawAxis(2);
      
    }

    // Limit the X,Y Rotation values so minor changes do not make motors move
    x =  Util.deadband(x, OI.kDeadband);//(Math.abs(x) > OI.kDeadband) ? x - OI.kDeadband : 0;
    y =  Util.deadband(y, OI.kDeadband);//(Math.abs(y) > OI.kDeadband) ? y - OI.kDeadband : 0;
    twist = Util.deadband(twist, OI.kDeadband);// (Math.abs(twist) > OI.kDeadband) ? twist - OI.kDeadband : 0;
    // Slew Rate Limit X,Y,Rotation values so drastic changes do not happen
    // x = xSRL.calculate(x);
    // y = ySRL.calculate(y);
    // twist = twistSRL.calculate(twist);
    if(RobotContainer.stickOperator.getRawButton(1)){
      RobotContainer.driveline.drive(x, y, twist, false);
    }else {
      RobotContainer.driveline.drive(x, y, twist, RobotContainer.driveline.getFieldOrientedModeActive());
    }
        //SmartDashboard.putNumber("drvX", x);
    //SmartDashboard.putNumber("drvY", y);
    SmartDashboard.putNumber("Gyro", RobotContainer.driveline.getRobotAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
