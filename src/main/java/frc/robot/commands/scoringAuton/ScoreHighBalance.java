// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoringAuton;

import frc.robot.Constants;
import frc.robot.commands.arm.MoveArmsNonPID;
import frc.robot.commands.balanceAuton.Balance;
import frc.robot.commands.gripper.MoveWristNonPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class ScoreHighBalance extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoreHighBalance(DriveSubsystem m_DriveSubsystem, ArmSubsystem m_ArmSubsystem, GripperSubsystem m_GripperSubsystem, SwerveControllerCommand smartSwerveController) {
    super(
        new MoveArmsNonPID(Constants.ArmConstants.kArm1HighAngle, Constants.ArmConstants.kArm2HighAngle, m_ArmSubsystem),
        new MoveWristNonPID(200, m_GripperSubsystem),
        new InstantCommand(()-> m_GripperSubsystem.activateGripper(), m_GripperSubsystem),
        new InstantCommand(() -> m_DriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
        new ParallelCommandGroup(new MoveWristNonPID(3, m_GripperSubsystem), new MoveArmsNonPID(Constants.ArmConstants.kArm1StartingAngle, Constants.ArmConstants.kArm2StartingAngle, m_ArmSubsystem), smartSwerveController),
        new Balance(m_DriveSubsystem)
    );

  }

}
