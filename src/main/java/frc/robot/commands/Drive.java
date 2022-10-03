// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  private DriveBase tDriveBase;

  /** Creates a new Drive. */
  public Drive(DriveBase drivebase) {
    tDriveBase = drivebase;
  }

  private ShuffleboardTab tab = Shuffleboard.getTab("mainTab");
  private NetworkTableEntry fullSpeed = tab.add("fullSpeed", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joyValueL = RobotContainer.m_joystick.getRawAxis(1);
    double joyValueR = RobotContainer.m_joystick.getRawAxis(5);
    if (fullSpeed.getBoolean(true)) {
      tDriveBase.move(ControlMode.PercentOutput, -joyValueL, -joyValueR);
    } else {
      tDriveBase.move(ControlMode.PercentOutput, -joyValueL * 0.5, -joyValueR * 0.5);
    }
    // tDriveBase.move(ControlMode.PercentOutput, -joyValueL, -joyValueR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tDriveBase.move(ControlMode.PercentOutput, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
