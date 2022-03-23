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
        public static final double kMaxAngularRateRadPerSecond = 4.324;//((1 / kWheelTrackBaseCircumference / 2 * Math.PI)
               // * kMaxSpeedMetersPerSecond) * 1;
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
        //https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
        // Starting kP of 0.01. Adjust higher to get closer to 10888 or lower to stop oscilations.
        public static final double kDriveMotionMagic_kP = 0.0;
        // Start with small value
        public static final double kDriveMotionMagic_kI = 0.0;
        // Starting kD is typically 10 x kP
        public static final double kDriveMotionMagic_kD = 0.0;
        // kF 50% speed. (0.50*1023) / (kMaxDriveVelUnitsPer100ms * 0.5) = 0.047
        public static final double kDriveMotionMagic_kF = 0.5; 
        // Set Cruise Velocit to 1/2 of max like kF. 
        public static final double kDriveMotionMagic_CruiseVel = 10888;
        // Set Accel to 1/2 of Max for 1 second ramp up.
        public static final double kDriveMotionMagic_Accel = 500;
        public static final int kDriveMotionMagic_Smoothing = 1;

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
        public static final double kDeadband = 0.13;
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
        public static final int kIntakeExtMotor = 31;
        public static final int kLiftLeft = 60;
        public static final int kLiftRight = 61;

    }
    public static final class PWMPORTS{
        public static final int kIntake = 5;
        public static final int kIntakeExt = 1;
        public static final int kLShooter = 2;
        public static final int kRShooter = 3;
    }

    public static final class INTAKE {
        public static final double kIntakeCargoDistanceLimit = 280;
    }

    public static final class SHOOTER {
        public static final double kBackHiSpeed = 0.65;
        public static final int kBackHiAngle = 0;

        public static final double kBackLowSpeed = 0.4;
        public static final int kBackLowAngle = 0;

        public static final double kFrontHiCloseSpeed = 0.62;
        public static final int kFrontHiCloseAngle = -110;

        public static final double kFrontLowCloseSpeed = 0.5;
        public static final int kFrontLowCloseAngle = -130;

        public static final double kFrontHi36InSpeed = 0.67;
        public static final int kFrontHi36InAngle = -140;

        public static final double kFrontHiLaunchSpeed = .8;
        public static final int kFrontHiLaunchAngle = -200;

        public static final double kFrontAutoLongSpeed = 0.8;
        public static final int kFrontAutoLongAngle = -150;

        public static final double kShotTimeBall1 = 2;
        public static final double kShotTimeBall2 = 2.5;

    }
    public static final class LIFT {
        public static final double kLimitUp = 200;
        public static final double kLimitDown = 50;  // Actually 50 below starting as positive
    }
    public static final class ROBOT {
        public static final double kFieldBlueCompassHeadingOffset = 0;
    }
}
