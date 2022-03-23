// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ShotEnum;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoT2C3S2C2S1 extends SequentialCommandGroup {
  public static String name = "T2 Cargo 3,2";
  /** Creates a new AutoT2C3S2C3S1. */
  public AutoT2C3S2C2S1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShooterShootCommand(ShotEnum.BACK_HI,2),
      // Drive Forward
      new AutoDriveCommand(0.35, 0, 60),
      new AutoDelayCommand(0.5),
      new AutoDriveCommand(-0.35, 0, 20),
      // Drive in Reverse
      new AutoRototeRobotCommand(0.30, -100, 3),
      new AutoIntakeExtCommand(),
      new AutoDriveCommand(0.35, 0, 110),
      new AutoDelayCommand(1),
      new AutoRototeRobotCommand(0.30, -130, 3),
      new AutoDriveCommand(0.35, 0, 10),
      new AutoShooterShootCommand(ShotEnum.FRONT_AUTO_LONG,3)


      // Shoot Back HI
      

      // Rotate to C2
      // Drive Forward
      // Drive Reverse
      // Rotate
      // Shoot
      // Drive out of Tarmac
      // Rotate to ball


    );
  }
}
