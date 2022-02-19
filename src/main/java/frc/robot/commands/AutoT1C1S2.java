// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
/**
 * Start in Tarmac 1, Get Cargo 1 and then Shoot 2 balls. 
 * Then point to the next ball.
 */
public class AutoT1C1S2 extends SequentialCommandGroup {
  public static String name = "T1 C1 S2";
  /** Creates a new AutoT1C1S2. */
  public AutoT1C1S2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
