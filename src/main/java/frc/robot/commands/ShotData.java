// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.ShotEnum;
import frc.robot.Constants.SHOOTER;

/** Add your docs here. */
public class ShotData {
    public static ShotEnum shot = ShotEnum.BACK_HI;
    public static int getAngle(){
        switch (shot) {
            case BACK_HI:
                return SHOOTER.kBackHiAngle;
            case BACK_LOW:
                return SHOOTER.kBackLowAngle;
            case FRONT_HI_CLOSE:
                return SHOOTER.kFrontHiCloseAngle;
            case FRONT_LOW_CLOSE:
                return SHOOTER.kFrontLowCloseAngle;
            case FRONT_HI_36INCH:
                return SHOOTER.kFrontHi36InAngle;
            case FRONT_HI_LAUNCHPAD:
                return SHOOTER.kFrontHiLaunchAngle;
            case FRONT_AUTO_LONG:
                return SHOOTER.kFrontAutoLongAngle;
            default:
                return SHOOTER.kFrontHiCloseAngle;
        }

    }
    public static double getSpeed(){
        switch (shot) {
            case BACK_HI:
                return SHOOTER.kBackHiSpeed;
            case BACK_LOW:
                return SHOOTER.kBackLowSpeed;
            case FRONT_HI_CLOSE:
                 return SHOOTER.kFrontHiCloseSpeed;
            case FRONT_LOW_CLOSE:
                return SHOOTER.kFrontLowCloseSpeed;
            case FRONT_HI_36INCH:
                return SHOOTER.kFrontHi36InSpeed;
            case FRONT_HI_LAUNCHPAD:
                return SHOOTER.kFrontHiLaunchSpeed;
            case FRONT_AUTO_LONG:
                return SHOOTER.kFrontAutoLongSpeed;
            default:
                return SHOOTER.kFrontHiCloseSpeed;
        }
    }
    public static String getShotString(){
        return "";
    }
}
