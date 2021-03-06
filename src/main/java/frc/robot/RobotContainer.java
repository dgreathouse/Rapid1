// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDoNothing;
import frc.robot.commands.AutoLeaveTarmac;
import frc.robot.commands.AutoT1C1;
import frc.robot.commands.AutoT1C4;
import frc.robot.commands.AutoT2C2;
import frc.robot.commands.AutoT2C3;
import frc.robot.commands.DrivelineDefaultCommand;
import frc.robot.commands.FieldOrientedModeActiveCommand;
import frc.robot.commands.FieldOrientedModeDisableCommand;
import frc.robot.commands.FieldOrientedResetCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeExtDefaultCommand;
import frc.robot.commands.IntakeExtDownCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.commands.LiftDefaultCommand;
import frc.robot.commands.LiftMoveCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShooterShootBackHiCommand;
import frc.robot.commands.ShooterShootBackLowCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.ShooterShootFrontExtraFarCommand;
import frc.robot.commands.ShooterShootFrontFarCommand;
import frc.robot.commands.ShooterShootFrontHiCommand;
import frc.robot.commands.ShooterShootFrontLowCommand;
import frc.robot.subsystems.Driveline;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExt;
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

  public static final IntakeExt intakeExt = new IntakeExt();
  private IntakeExtDefaultCommand intakeExtDefaultCommand = new IntakeExtDefaultCommand(intakeExt);

  public static final Shooter shooter = new Shooter();
  private ShooterDefaultCommand shooterDefaultCommand = new ShooterDefaultCommand(shooter);

  public static final Lift lift = new Lift();
  private LiftDefaultCommand liftDefaultCommand = new LiftDefaultCommand(lift);

  public static Joystick stickDriver = new Joystick(0);
  public static Joystick stickOperator = new Joystick(1);

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static final PneumaticsControlModule PCM = new PneumaticsControlModule();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveline.setDefaultCommand(drivelineDefaultCommand);
    intake.setDefaultCommand(intakeDefaultCommand);
    shooter.setDefaultCommand(shooterDefaultCommand);
    lift.setDefaultCommand(liftDefaultCommand);
    intakeExt.setDefaultCommand(intakeExtDefaultCommand);
    // Configure the button bindings
    configureButtonBindings();

    autoChooser.addOption(AutoLeaveTarmac.name, new AutoLeaveTarmac());
    autoChooser.addOption(AutoT1C1.name, new AutoT1C1());
    autoChooser.addOption(AutoT1C4.name, new AutoT1C4());
    autoChooser.addOption(AutoT2C2.name, new AutoT2C2());
    autoChooser.addOption(AutoT2C3.name, new AutoT2C3());
    autoChooser.setDefaultOption(AutoDoNothing.name, new AutoDoNothing());

    SmartDashboard.putData(autoChooser);
    UsbCamera cam =  CameraServer.startAutomaticCapture();
    cam.setFPS(15);
    cam.setResolution(360, 240);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Operator buttons
    JoystickButton shotTrigger = new JoystickButton(stickOperator, 1);
    shotTrigger.whileHeld(new ShooterShootCommand());

    JoystickButton intakeOut1Btn = new JoystickButton(stickOperator, 4);
    intakeOut1Btn.whileHeld(new IntakeSpinCommand(-1));

    JoystickButton intakeOut2Btn = new JoystickButton(stickOperator, 3);
    intakeOut2Btn.whileHeld(new IntakeSpinCommand(-1));

    JoystickButton intakeIn1Btn = new JoystickButton(stickOperator, 5);
    intakeIn1Btn.whileHeld(new IntakeSpinCommand(1));

    JoystickButton intakeIn2Btn = new JoystickButton(stickOperator, 6);
    intakeIn2Btn.whileHeld(new IntakeSpinCommand(1));

    JoystickButton ShooterFrontHiBtn = new JoystickButton(stickOperator, 8);
    ShooterFrontHiBtn.whenPressed(new ShooterShootFrontHiCommand());

    JoystickButton ShooterFrontLowBtn = new JoystickButton(stickOperator, 7);
    ShooterFrontLowBtn.whenPressed(new ShooterShootFrontLowCommand());

    JoystickButton ShooterBackHiBtn = new JoystickButton(stickOperator, 12);
    ShooterBackHiBtn.whenPressed(new ShooterShootBackHiCommand());

    JoystickButton ShooterBackLowBtn = new JoystickButton(stickOperator, 11);
    ShooterBackLowBtn.whenPressed(new ShooterShootBackLowCommand());

    JoystickButton ShooterFrontFarBtn = new JoystickButton(stickOperator, 9);
    ShooterFrontFarBtn.whenPressed(new ShooterShootFrontFarCommand());

    JoystickButton ShooterFrontExtraFar = new JoystickButton(stickOperator, 10);
    ShooterFrontExtraFar.whenPressed(new ShooterShootFrontExtraFarCommand());

    // Driver buttons
    JoystickButton intakeExtBtn = new JoystickButton(stickDriver, 1);
    intakeExtBtn.whileHeld(new IntakeExtDownCommand());
    
    JoystickButton liftMoveBtn = new JoystickButton(stickDriver, 10);
    liftMoveBtn.whileHeld(new LiftMoveCommand());

    JoystickButton resetGyroBtn = new JoystickButton(stickDriver, 12);
    resetGyroBtn.whenPressed(new FieldOrientedResetCommand());

    JoystickButton DrivelineFieldOrientedModeActive = new JoystickButton(stickDriver, 7);
    DrivelineFieldOrientedModeActive.whenPressed(new FieldOrientedModeActiveCommand());

    JoystickButton DrivelineFieldOrientedModeDisable = new JoystickButton(stickDriver, 8);
    DrivelineFieldOrientedModeDisable.whenPressed(new FieldOrientedModeDisableCommand());

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
