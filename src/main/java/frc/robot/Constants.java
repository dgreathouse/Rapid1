// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class COMM {
        public static final int kTimeoutMs = 30;
    }

    public static final class DRIVE {
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = 0.56;
        // Distance between front and back wheels on robot
        public static final double kWheelBase = 0.56;
        public static final double kWheelTrackBaseCircumference = 2 * Math.PI
                * (Math.sqrt(Math.pow((kTrackWidth / 2), 2) + Math.pow((kWheelBase / 2), 2)));
        // Converts units/100ms to wheel m/sec
        // Units/100ms max measured = 22,036
        public static final double kDriveVelRatio = 503.6815;
        public static final double kMaxDriveVelUnitsPer100ms = 21777.0;
        public static final double kMaxSpeedMetersPerSecond = 4.324;
        public static final double kMaxAngularRateRadPerSecond = ((1 / kWheelTrackBaseCircumference / 2 * Math.PI)
                * kMaxSpeedMetersPerSecond) * 1;
        // frontLeft, frontRight, rearLeft, rearRight
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    }

    public static final class SWERVE {

        public static final double kDriveMotEncoderCountsPerRev = 2048.0;
        public static final double kDriveRatio = 7.85;
        public static final double kWheelDiameter = 4.0;
        public static final double kDriveCntsPerInch = kDriveMotEncoderCountsPerRev * kDriveRatio / (Math.PI * kWheelDiameter);
        public static final int kDriveAutoMotionMagicSlotIdx = 0;
        public static final double kDriveMotionMagic_kP = 0;
        public static final double kDriveMotionMagic_kI = 0;
        public static final double kDriveMotionMagic_kD = 0;
        public static final double kDriveMotionMagic_kF = 0;



        public static final double kSteerMotEncoderCountsPerRev = 2048.0;
        public static final double kSteerRatio = 15.43;
        public static final double kSteerMotCntsPerWheelDeg = (kSteerMotEncoderCountsPerRev * kSteerRatio) / 360;
        public static final double kSteerMotCountsPerWheelRadian = (kSteerMotEncoderCountsPerRev / (2 * Math.PI)) * kSteerRatio;
        public static final double kSteerEncoderCountsPerRev = 4096.0;
        public static final double kSteerCountsPerRadian = kSteerEncoderCountsPerRev / 2 * Math.PI;

        public static final double kLFAbsoluteOffsetInDegrees = 139.5;
        public static final double kRFAbsoluteOffsetInDegrees = 217.5;
        public static final double kLBAbsoluteOffsetInDegrees = 332.6;
        public static final double kRBAbsoluteOffsetInDegrees = 46.5;
    }

    public static final class OI {
        public static final double kDeadband = 0.1;
    }

    public static final class CANIDS {
        public static final int kDriveline_LFSteer = 10;
        public static final int kDriveline_LFDrive = 14;
        public static final int kDriveline_LFSteerEnc = 3;
        public static final int kDriveline_LBSteer = 12;
        public static final int kDriveline_LBDrive = 16;
        public static final int kDriveline_LBSteerEnc = 5;
        public static final int kDriveline_RFSteer = 11;
        public static final int kDriveline_RFDrive = 15;
        public static final int kDriveline_RFSteerEnc = 4;
        public static final int kDriveline_RBSteer = 13;
        public static final int kDriveline_RBDrive = 17;
        public static final int kDriveline_RBSteerEnc = 6;

        public static final int kShooterLeft = 21;
        public static final int kShooterRight = 20;
        public static final int kShooterAngle = 23;

        public static final int kIntake = 30;
    }

    public static final class INTAKE {
        public static final double kIntakeCargoDistanceLimit = 280;
    }

    public static final class SHOOTER {
        public static final double kBackHiSpeed = 1;
        public static final int kBackHiAngle = 0;

        public static final double kBackLowSpeed = 0.9;
        public static final int kBackLowAngle = -40;

        public static final double kFrontHiCloseSpeed = 0.9;
        public static final int kFrontHiCloseAngle = -60;

        public static final double kFrontLowCloseSpeed = 0.9;
        public static final int kFrontLowCloseAngle = -80;

        public static final double FRONT_HI_36IN_SPEED = 0.9;
        public static final int FRONT_HI_36IN_ANGLE = -100;

        public static final double FRONT_HI_LAUNCH_SPEED = 0.9;
        public static final int FRONT_HI_LAUNCH_ANGLE = -150;

    }
}