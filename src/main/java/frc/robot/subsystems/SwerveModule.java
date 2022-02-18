// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.COMM;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.SWERVE;

/** Add your docs here. */
/*
 * By default all steer motors turn clockwise with a positive input and encoder
 * is positive when looking down on the module.
 * In Steer math clockwise should be negative so all modules need to be
 * Inverted.
 */
public class SwerveModule {
    private final WPI_TalonFX m_steerMotor;
    private final WPI_TalonFX m_driveMotor;
    private final CANCoder m_steerEncoder;
    private double m_steerEncoderAbsoluteOffset;
    private double m_driveSpeed = 0;

    private int[][] m_desiredQuadDirIndexArray = { { 0, 2, 1, 3 }, { 1, 0, 3, 2 }, { 2, 3, 0, 1 }, { 3, 1, 2, 0 } };
    private int[][] m_virtualQuadArray = { { 1, 2, 3, 4, -1, -2, -3, -4 }, { 2, 3, 4, 1, 1, -1, -2, -3 },
            { -1, 1, 2, 3, -2, -3, -4, -1 }, { -2, -1, 1, 2, 2, 1, -1, -2 } };
    private int m_virtualQuad = 1;
    private int m_previousDesiredQuad = 1;
    private String name = "";
    public SwerveModule(String _name, int _steerCANID, int _driveCANID, int _steerCANCoderCANID, InvertType _drvInvert, InvertType _strInvert, double _angOffsetDeg) {
        name = _name;
        m_driveMotor = new WPI_TalonFX(_driveCANID);
        m_driveMotor.configFactoryDefault();
        m_driveMotor.setInverted(_drvInvert);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configOpenloopRamp(0.1);
        m_driveMotor.configClosedloopRamp(0.1);

        // Initialize slot 1 for MotionMagic Autonomous drive
        m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        m_driveMotor.configNeutralDeadband(0.001, 30);
        m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, COMM.kTimeoutMs);
        m_driveMotor.configNominalOutputForward(0,COMM.kTimeoutMs);
        m_driveMotor.configNominalOutputReverse(0, COMM.kTimeoutMs);
        m_driveMotor.configPeakOutputForward(1, COMM.kTimeoutMs);
        m_driveMotor.configPeakOutputReverse(-1, COMM.kTimeoutMs);
        m_driveMotor.selectProfileSlot(SWERVE.kDriveAutoMotionMagicSlotIdx, 0);
        m_driveMotor.config_kP(SWERVE.kDriveAutoMotionMagicSlotIdx, SWERVE.kDriveMotionMagic_kP, COMM.kTimeoutMs);
        m_driveMotor.config_kI(SWERVE.kDriveAutoMotionMagicSlotIdx, SWERVE.kDriveMotionMagic_kP, COMM.kTimeoutMs);
        m_driveMotor.config_kD(SWERVE.kDriveAutoMotionMagicSlotIdx, SWERVE.kDriveMotionMagic_kP, COMM.kTimeoutMs);
        m_driveMotor.config_kF(SWERVE.kDriveAutoMotionMagicSlotIdx, SWERVE.kDriveMotionMagic_kP, COMM.kTimeoutMs);
        m_driveMotor.configMotionCruiseVelocity(SWERVE.kDriveMotionMagic_CruiseVel, COMM.kTimeoutMs);
        m_driveMotor.configMotionAcceleration(SWERVE.kDriveMotionMagic_Accel, COMM.kTimeoutMs);



        m_steerMotor = new WPI_TalonFX(_steerCANID);
        m_steerMotor.setInverted(_strInvert);
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
        m_steerMotor.config_kP(0, .105);
      //  m_steerMotor.closedLoop(0, 1);
        m_steerMotor.configClosedloopRamp(0.1);
        m_steerEncoderAbsoluteOffset = _angOffsetDeg;
        

        // Set steerMotor encoder position to difference between current abs and calibration that is straight ahead

        m_steerEncoder = new CANCoder(_steerCANCoderCANID);
        double steerAng = m_steerEncoder.getAbsolutePosition();
        double steerDiff =  (steerAng - _angOffsetDeg) * SWERVE.kSteerMotCntsPerWheelDeg;
       //SmartDashboard.putNumber(Double.toString(_angOffsetDeg), steerDiff);
        m_steerMotor.setSelectedSensorPosition(steerDiff);


    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityInMPS(), new Rotation2d(getSteerEncAngleInRad()));
    }
    public void resetDriveMotorCnts(){
        m_driveMotor.setSelectedSensorPosition(0);
    }
    
    public void resetSteerMotorCnts(){
        m_steerMotor.setSelectedSensorPosition(0);
    }
    public double getSteerMotorCnts(){
        return m_steerMotor.getSelectedSensorPosition();
    }
    public double getDriveMotorCnts(){
        return m_driveMotor.getSelectedSensorPosition();
    }
    public double getSteerMotAngInRad() {
        double motCnts =  m_steerMotor.getSelectedSensorPosition();
        motCnts = motCnts % 2047;
        return motCnts / SWERVE.kSteerMotCountsPerWheelRadian;
    }
    public double getSteerEncAngleInRad() {
        return m_steerEncoder.getPosition() * Math.PI / 180.0;
    }
    public double getSteerEncAngleDeg(){
        return m_steerEncoder.getAbsolutePosition();
    }
    public Rotation2d getRotation2d() {
        return new Rotation2d(getSteerMotAngInRad());
    }

    public double getDriveVelocityInMPS() {
        return m_driveMotor.getSelectedSensorVelocity(0) * DRIVE.kDriveVelRatio;
    }

    public double getDriveSpeed(){
        return m_driveSpeed;
    }
    // Calculate the states and command the motors.
    public void setDesiredState(SwerveModuleState _desiredState) {
        //calcNewAngle(_desiredState);
    //  SwerveModuleState swerveModuleState = _desiredState;
      SwerveModuleState swerveModuleState = SwerveModuleState.optimize(_desiredState, getRotation2d());
      double steerAng = swerveModuleState.angle.getRadians();
      double steerCnts = steerAng * SWERVE.kSteerMotCountsPerWheelRadian;
      m_steerMotor.set(ControlMode.Position, steerCnts);
      
      double driveSpeed = swerveModuleState.speedMetersPerSecond;
      driveSpeed = driveSpeed / DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.set(ControlMode.PercentOutput, driveSpeed);
      m_driveSpeed = driveSpeed;
     // SmartDashboard.putNumber(name + "_Vel", m_driveMotor.getSelectedSensorVelocity());
    }

    public void setDesiredState(SwerveModuleState _desiredState, boolean disableDrive){
        SwerveModuleState swerveModuleState = _desiredState;
        double steerAng = swerveModuleState.angle.getRadians();
        double steerCnts = steerAng * SWERVE.kSteerMotCountsPerWheelRadian;
        m_steerMotor.set(ControlMode.Position, steerCnts);

        if(!disableDrive){
            double driveSpeed = swerveModuleState.speedMetersPerSecond / DRIVE.kMaxSpeedMetersPerSecond;
            m_driveMotor.set(ControlMode.PercentOutput, driveSpeed);
        }else {
            m_driveMotor.set(ControlMode.PercentOutput, 0);
        }
        
    }
    /** Use MotionMagic of the FalconFX to drive the 4 drive motors to a distance
     * Initialization sets up A and V for the Motors.
     * The Steer motor needs to be done before this.
     * 
     * @param distanceInches Distance in Inches
     */
    public void setDesiredStateAutoMotionMagic(SwerveModuleState _desiredState, double _distanceInches){
        SwerveModuleState swerveModuleState = _desiredState;
        double steerAng = swerveModuleState.angle.getRadians();
        double steerCnts = steerAng * SWERVE.kSteerMotCountsPerWheelRadian;
        m_steerMotor.set(ControlMode.Position, steerCnts);

        
        m_driveMotor.selectProfileSlot(SWERVE.kDriveAutoMotionMagicSlotIdx, COMM.kTimeoutMs);  // MotionMagic PID in slot 1.
        m_driveMotor.set(TalonFXControlMode.MotionMagic, _distanceInches * SWERVE.kDriveCntsPerInch);
    }
    private void calcNewAngle(SwerveModuleState _state) {
        double newAng;
        int quad = 0;

        quad = calcAngleQuadrant(_state.angle.getDegrees());
        if (quad > 2) {
            newAng = Math.toRadians(180 + (180 + _state.angle.getDegrees()));
        } else if (quad < -2) {
            newAng = Math.toRadians(-180 + (-180 + _state.angle.getDegrees()));
        } else {
            newAng = _state.angle.getRadians();
        }

        _state.angle = new Rotation2d(newAng);
    }

    private int calcQuad(double _desiredAngle) {
        int desiredQuad = 1;
        if (_desiredAngle >= 0 && _desiredAngle < 90) {
            desiredQuad = 1;
        } else if (_desiredAngle >= 90 && _desiredAngle < 180) {
            desiredQuad = 2;
        } else if (_desiredAngle < 0 && _desiredAngle > -90) {
            desiredQuad = -1;
        } else if (_desiredAngle < -90 && _desiredAngle > -180) {
            desiredQuad = -2;
        }
        return desiredQuad;
    }

    private int calcAngleQuadrant(double _desiredAngle) {
        int desiredQuad = calcQuad(_desiredAngle);
        int desiredQuadDirectionIndex = Math.abs(desiredQuad - 1);
        int previousDesiredQuadDirectionIndex = Math.abs(m_previousDesiredQuad - 1);
        int desiredQuadDirection = m_desiredQuadDirIndexArray[desiredQuadDirectionIndex][previousDesiredQuadDirectionIndex];
        int virtualQuadIndex = m_virtualQuad < 0 ? Math.abs(m_virtualQuad - 3) : Math.abs(m_virtualQuad - 1);
        m_virtualQuad = m_virtualQuadArray[desiredQuadDirection][virtualQuadIndex];
        m_previousDesiredQuad = desiredQuad;
        return m_virtualQuad;
    }
   
}
