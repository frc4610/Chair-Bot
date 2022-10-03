// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
  public TalonSRX driveFrontR;
  public TalonSRX driveFrontL;
  public TalonSRX driveBackR;
  public TalonSRX driveBackL;
  public DriveBase() {
    driveFrontR = new TalonSRX(Constants.TALONSRX_FR_ID);
    driveFrontL = new TalonSRX(Constants.TALONSRX_FL_ID);
    driveBackR = new TalonSRX(Constants.TALONSRX_BR_ID);
    driveBackL = new TalonSRX(Constants.TALONSRX_BL_ID);
    driveFrontR.configOpenloopRamp(.4);
    driveFrontL.configOpenloopRamp(.4);
    driveBackR.configOpenloopRamp(.4);
    driveBackL.configOpenloopRamp(.4);
    driveFrontR.setInverted(true);
  }
  public void move(ControlMode mode, double speedL, double speedR) {
    driveFrontR.set(mode, speedR);
    driveFrontL.set(mode, speedL);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
