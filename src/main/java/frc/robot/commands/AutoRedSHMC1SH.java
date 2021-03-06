// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRedSHMC1SH extends SequentialCommandGroup {
  /** Creates a new AutoRedSHMC1SH. 
   * Shoot Hi Move to Cargo 1 Shoot Hi
  */
  public AutoRedSHMC1SH() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveCommand(0.5, 0.0, 192),
      new AutoDriveCommand(0.4, 0.0, -192)
      
    );
  }
}
