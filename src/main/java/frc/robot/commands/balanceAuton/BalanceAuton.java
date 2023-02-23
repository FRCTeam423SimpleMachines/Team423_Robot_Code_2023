// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balanceAuton;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class BalanceAuton extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BalanceAuton(DriveSubsystem m_DriveSubsystem, SwerveControllerCommand smartSwerveController, SwerveControllerCommand smartServeController2) {
    super(
        new InstantCommand(() -> m_DriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
        smartSwerveController,
        new InstantCommand(() -> m_DriveSubsystem.drive(0, 0, 0, false, false)),
        new InstantCommand(() -> m_DriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
        smartServeController2,
        new InstantCommand(() -> m_DriveSubsystem.drive(0, 0, 0, false, false))

    );

  }

}
