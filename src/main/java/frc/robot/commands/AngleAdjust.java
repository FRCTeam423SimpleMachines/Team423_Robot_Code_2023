// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AngleAdjust extends CommandBase {
  double Kp = -0.1f;
  double min_command = 0.05f;
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  double tx;
  double headingError;
  double steeringAdjust = 0.0f;
  VisionSubsystem vision;
  DriveSubsystem drive;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AngleAdjust(VisionSubsystem vision, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drive = drive;
    addRequirements(vision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = limelightTable.getEntry("tx").getDouble(0);
    headingError = -tx;

    if (Math.abs(headingError) > 1.0) {
      if (headingError < 0) {
        steeringAdjust = Kp*headingError + min_command;
      } else {
        steeringAdjust = Kp*headingError - min_command;
      }
    }
    else {
      steeringAdjust = 0;
    }
    drive.drive(0.0, 0.0, steeringAdjust, false, true);
}


  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (steeringAdjust == 0) {
      return true;
    }
    else {
      return false;
    }
  }
}