// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

public class AutoRototeRobotCommand extends PIDCommand {
  double m_rotationSpeed, m_angDeg, m_timeOut;
  Timer timer;
  boolean isFinished = false;

  /** Creates a new AutoRototeRobotCommand. */
  public AutoRototeRobotCommand(double _rotationSpeed, double _angleDeg, double _timeOut) {
    super(
        // The controller that the command will use
        new PIDController(.15, 0.01, 0),
        // This should return the measurement
        () -> RobotContainer.driveline.getRobotAngle(),
        // This should return the setpoint (can also be a constant)
        () -> _angleDeg,
        // This uses the output
        output -> {
          double out = output;
          if(out > 3){ out = 3;}
          RobotContainer.driveline.autoRotateRobot(out);;
        });
    addRequirements(RobotContainer.driveline);
    m_controller.setTolerance(.5,100);
   
    m_rotationSpeed = _rotationSpeed;
    m_angDeg = _angleDeg;
    m_timeOut = _timeOut;
  }

  @Override
  public void initialize(){
    RobotContainer.driveline.resetGyro();
    super.initialize();
    timer = new Timer();
    timer.start();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.m_controller.atSetpoint() || timer.hasElapsed(m_timeOut)){
      RobotContainer.driveline.drive(0, 0, 0, false);
      return true;
    }else return false;
    
  }
}
