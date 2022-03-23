// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SWERVE;

public class Driveline extends SubsystemBase {
  private final SwerveModule m_leftFront = new SwerveModule("LF", CANIDS.kDriveline_LFSteer, CANIDS.kDriveline_LFDrive,
      CANIDS.kDriveline_LFSteerEnc, InvertType.None, InvertType.InvertMotorOutput, SWERVE.kLFAbsoluteOffsetInDegrees);
  private final SwerveModule m_rightFront = new SwerveModule("RF", CANIDS.kDriveline_RFSteer, CANIDS.kDriveline_RFDrive,
      CANIDS.kDriveline_RFSteerEnc, InvertType.InvertMotorOutput, InvertType.InvertMotorOutput,
      SWERVE.kRFAbsoluteOffsetInDegrees);
  private final SwerveModule m_leftBack = new SwerveModule("LB", CANIDS.kDriveline_LBSteer, CANIDS.kDriveline_LBDrive,
      CANIDS.kDriveline_LBSteerEnc, InvertType.None, InvertType.InvertMotorOutput, SWERVE.kLBAbsoluteOffsetInDegrees);
  private final SwerveModule m_rightBack = new SwerveModule("RB", CANIDS.kDriveline_RBSteer, CANIDS.kDriveline_RBDrive,
      CANIDS.kDriveline_RBSteerEnc, InvertType.InvertMotorOutput, InvertType.InvertMotorOutput,
      SWERVE.kRBAbsoluteOffsetInDegrees);

  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);
  
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DRIVE.kDriveKinematics, m_gyro.getRotation2d());
  private boolean isFieldOrientedMode = false;
  /** Creates a new Driveline. */
  public Driveline() {

  }

  @Override
  public void periodic() {
    // Update the robots position on the field. This is used for Autonomous.
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_leftFront.getState(),
        m_leftBack.getState(),
        m_rightFront.getState(),
        m_rightBack.getState());

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    m_gyro.getAngle();
  }

  /**
   * Method to drive the robot using joystick info.
   * Inputs are +/- 1 from the joystick
   * Values are converted to needed swereve module rates
   *
   * @param _xSpeed        Speed of the robot in the x direction (forward).
   * @param _ySpeed        Speed of the robot in the y direction (sideways).
   * @param _rot           Angular rate of the robot.
   * @param _fieldRelative Whether the provided x and y speeds are relative to the
   *                       field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
    // Convert joystick values of +/- 1 to Meters/Sec and Rad/Sec
    // Joystick Y/X axis are reveresed here. Joystick Y is pushing forward and back.
    // The kinematics assumes X to be forward and back.
    double xSpeed = _ySpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double rot = -_rot * DRIVE.kMaxAngularRateRadPerSecond;
    // rot = 0;

    // Calculate the swerve module states
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics.toSwerveModuleStates(_fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    // Set the desired state of each swerve module with the new calculated states.
    m_leftFront.setDesiredState(swerveModuleStates[0]);
    m_rightFront.setDesiredState(swerveModuleStates[1]);
    m_leftBack.setDesiredState(swerveModuleStates[2]);
    m_rightBack.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Autonomous Drive to inches using the PID of the TalonFX
   * The Angle must be set first so a movement does not happen before wheel is
   * fully turned
   * The best way to get the robot to drive straight is to put drive motors in a
   * velocity loop for a set time.
   * 
   * @param _xSpeed     X Speed in MPS Fwd +
   * @param _ySpeed     Y Speed in MPS Left +
   * @param _distanceIn Distances to travel in inches
   */
  public void autoDrive(double _xSpeed, double _ySpeed, double _distanceIn) {
    double xSpeed = _xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_ySpeed * DRIVE.kMaxSpeedMetersPerSecond;

    double rot = RobotContainer.driveline.getRobotAngle() * 0.001;
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    m_leftFront.setDesiredStateAutoMotionMagic(swerveModuleStates[0], _distanceIn);
    m_rightFront.setDesiredStateAutoMotionMagic(swerveModuleStates[1], _distanceIn);
    m_leftBack.setDesiredStateAutoMotionMagic(swerveModuleStates[2], _distanceIn);
    m_rightBack.setDesiredStateAutoMotionMagic(swerveModuleStates[3], _distanceIn);


  }
  public void autoDrive(double _xSpeed, double _ySpeed) {
    double xSpeed = _xSpeed * DRIVE.kMaxSpeedMetersPerSecond;
    double ySpeed = -_ySpeed * DRIVE.kMaxSpeedMetersPerSecond;

    double rot = RobotContainer.driveline.getRobotAngle() * 0.025;
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    m_leftFront.setDesiredState(swerveModuleStates[0], false);
    m_rightFront.setDesiredState(swerveModuleStates[1], false);
    m_leftBack.setDesiredState(swerveModuleStates[2], false);
    m_rightBack.setDesiredState(swerveModuleStates[3], false);


  }
  /**
   * Autonomous Rotate wheel while driving speeds disabled
   * 
   * @param _xSpeed X Speed in MPS Fwd +
   * @param _ySpeed Y Speed in MPS Left +
   */
  public void autoRotateWheels(double _xSpeed, double _ySpeed) {
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(_xSpeed, _ySpeed, 0));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    m_leftFront.setDesiredState(swerveModuleStates[0], true);
    m_rightFront.setDesiredState(swerveModuleStates[1], true);
    m_leftBack.setDesiredState(swerveModuleStates[2], true);
    m_rightBack.setDesiredState(swerveModuleStates[3], true);
  }

  public void autoRotateRobot(double _rot) {
    SwerveModuleState[] swerveModuleStates = DRIVE.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, _rot));
    // Normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DRIVE.kMaxSpeedMetersPerSecond);
    m_leftFront.setDesiredState(swerveModuleStates[0], false);
    m_rightFront.setDesiredState(swerveModuleStates[1], false);
    m_leftBack.setDesiredState(swerveModuleStates[2], false);
    m_rightBack.setDesiredState(swerveModuleStates[3], false);
  }

  public void resetSwerveSteerEncoders() {
    m_leftFront.resetSteerMotorCnts();
    m_rightFront.resetSteerMotorCnts();
    m_leftBack.resetSteerMotorCnts();
    m_rightBack.resetSteerMotorCnts();
  }

  public void resetSwerveDriveEncoders() {
    m_leftFront.resetDriveMotorCnts();
    m_rightFront.resetDriveMotorCnts();
    m_leftBack.resetDriveMotorCnts();
    m_rightBack.resetDriveMotorCnts();
  }
  public double getAverageDistanceInInches(){
    double dis = (m_leftFront.getDriveMotorCnts() + m_leftBack.getDriveMotorCnts() + m_rightBack.getDriveMotorCnts() + m_rightFront.getDriveMotorCnts()) / 4.0;
    return dis / SWERVE.kDriveCntsPerInch;
  }
  public double getAverageVelocity(){
    double vel = (m_leftFront.getDriveVelocityInNativeUnits() + m_leftBack.getDriveVelocityInNativeUnits() + m_rightBack.getDriveVelocityInNativeUnits() + m_rightFront.getDriveVelocityInNativeUnits()) / 4.0;
    return vel;
  }
  public double getRobotAngle(){
    return -m_gyro.getAngle();
  }
  public double getRobotRoll(){
    
    return m_gyro.getPitch();
  }
  public void resetGyro(){
    m_gyro.reset();
  }
  public void setFieldOrientedMode(boolean mode){
    isFieldOrientedMode = mode;
  }
  public boolean getFieldOrientedModeActive(){
    return isFieldOrientedMode;
  }
}
