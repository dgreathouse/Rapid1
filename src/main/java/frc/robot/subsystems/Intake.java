// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.INTAKE;

public class Intake extends SubsystemBase {
  VictorSPX motor = new VictorSPX(CANIDS.kIntake);
  Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
  /** Creates a new Intake. */
  public Intake() {
    distanceSensor.setEnabled(true);
    distanceSensor.setAutomaticMode(true);
    distanceSensor.setDistanceUnits(Unit.kMillimeters);
  }

  public boolean isCargoInIntake(){
    if(distanceSensor.getRange() < INTAKE.kIntakeCargoDistanceLimit){
      return true;
    }else {
      return false;
    }
  }
  public void spin(double _speed){
    motor.set(ControlMode.PercentOutput, _speed);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Distance Sensor (mm)", distanceSensor.getRange());
    
  }
}
