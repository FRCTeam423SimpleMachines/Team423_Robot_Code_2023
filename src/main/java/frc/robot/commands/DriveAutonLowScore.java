package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.arm.MoveArm1NonPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAutonLowScore extends SequentialCommandGroup{
    
    
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveAutonLowScore(DriveSubsystem m_DriveSubsystem, ArmSubsystem m_ArmSubsystem, SwerveControllerCommand smartSwerveController) {
    super(
        new MoveArm1NonPID(90, m_ArmSubsystem),
        new InstantCommand(() -> m_DriveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
        smartSwerveController
    );

  }
    
}
