// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDoNothing;
import frc.robot.commands.AutoRedSHMC1SH;
import frc.robot.commands.DrivelineDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.commands.LiftDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.subsystems.Driveline;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static final Driveline driveline = new Driveline();
  private DrivelineDefaultCommand drivelineDefaultCommand = new DrivelineDefaultCommand(driveline);

  public static final Intake intake = new Intake();
  private IntakeDefaultCommand  intakeDefaultCommand = new IntakeDefaultCommand(intake);

  public static final Shooter shooter = new Shooter();
  private ShooterDefaultCommand shooterDefaultCommand = new ShooterDefaultCommand(shooter);

  public static final Lift lift = new Lift();
  private LiftDefaultCommand liftDefaultCommand = new LiftDefaultCommand(lift);

  public static Joystick stickDriver = new Joystick(0);
  public static Joystick stickOperator = new Joystick(1);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveline.setDefaultCommand(drivelineDefaultCommand);
    intake.setDefaultCommand(intakeDefaultCommand);
    shooter.setDefaultCommand(shooterDefaultCommand);
    lift.setDefaultCommand(liftDefaultCommand);
    // Configure the button bindings
    configureButtonBindings();

    autoChooser.setDefaultOption("Red ShootHi Cargo 1 SH", new AutoRedSHMC1SH());
    autoChooser.addOption("Do Nothing", new AutoDoNothing());

    SmartDashboard.putData(autoChooser);
    //CameraServer.startAutomaticCapture();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton shotTrigger = new JoystickButton(stickOperator, 1);
    shotTrigger.whileHeld(new ShooterShootCommand());

    JoystickButton intakeInBtn = new JoystickButton(stickOperator, 2);
    intakeInBtn.whileHeld(new IntakeSpinCommand(0.5));

    JoystickButton intakeOutBtn = new JoystickButton(stickOperator, 3);
    intakeOutBtn.whileHeld(new IntakeSpinCommand(-0.5));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}