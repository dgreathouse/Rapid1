// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ShotEnum;
import frc.robot.Constants.SHOOTER;

public class AutoT1C4 extends SequentialCommandGroup {
  public static String name = "T1 Cargo 4";
  /** Creates a new AutoT1C4. */
  public AutoT1C4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveCommand(-0.35, 0, 50),
      
      new AutoShooterShootCommand(ShotEnum.FRONT_AUTO_LONG, SHOOTER.kShotTimeBall2),
      new AutoRototeRobotCommand(0.30, 30, 3),
      new AutoDriveCommand(-0.35, 0, 30),
      new FieldOrientedResetCommand(),
      new FieldOrientedModeActiveCommand()
    );
  }
}
