package frc.robot.commands.balanceAuton;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalancePath1 {

    public static SwerveControllerCommand returnController(DriveSubsystem m_DriveSubsystem){
            // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        1,
        2.0)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        List.of(new Translation2d(1, 0.1), new Translation2d(4.5, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        //new Pose2d(4.5, 0, new Rotation2d(0)),
        new Pose2d(2.7, 0.0, new Rotation2d(0)),
        config);

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new SwerveControllerCommand(
        exampleTrajectory,
        m_DriveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_DriveSubsystem::setModuleStates,
        m_DriveSubsystem);


    }

    
}