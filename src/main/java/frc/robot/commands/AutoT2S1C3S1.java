// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ShotEnum;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoT2S1C3S1 extends SequentialCommandGroup {
  public static String name = "T2 S1 C3 S1";
  /** Creates a new AutoT2S1C3S1. */
  public AutoT2S1C3S1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShooterShootCommand(ShotEnum.BACK_HI,2),
      // Drive Forward
      new AutoDriveCommand(0.35, 0, 62),
      new AutoDelayCommand(0.5),
      new AutoDriveCommand(-0.35, 0, 72),
      new AutoShooterShootCommand(ShotEnum.BACK_HI,2),
      new AutoRototeRobotCommand(0.30, -30, 3),
      
      new AutoDriveCommand(0.35, 0, 72)
    );
  }
}
