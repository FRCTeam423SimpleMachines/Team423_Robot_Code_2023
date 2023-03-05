package frc.robot.commands.balanceAuton;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.subsystems.DriveSubsystem;

public class BalancePID extends ProfiledPIDCommand{

    /**
    * Turns to robot to the specified angle using a motion profile.
    *
    * @param targetAngleDegrees The angle to turn to
    * @param drive              The drive subsystem to use
    */

    DriveSubsystem m_drive;
    private int count = 0;
    private double p = 1;
    private double i = 0;
    private double d = 0.5;

    public BalancePID(DriveSubsystem drive) {

        super(
            new ProfiledPIDController(
                1, 
                0,
                0.5, 
                new TrapezoidProfile.Constraints(
                    0.3,
                    0.02
                )
            ),
            // Close loop on heading
            () -> drive.getPitch(),
            // Set reference to target
            0,
            // Pipe output to turn robot
            (output, setpoint) -> drive.drive(-output, 0, 0, true, false, true),
            
            // Require the drive
            drive
        );

        m_drive = drive;
        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(3, 1);

    }

    @Override
    public void initialize() {

        // TODO Auto-generated method stub
        getController().reset(m_drive.getPitch());
        getController().setPID(p, i, d);

        count = 0;
        super.initialize();
    

    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        m_drive.drive(0.0, 0.0, 0, true, false);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        SmartDashboard.putNumber("TurnPID/PosError", getController().getPositionError());
        SmartDashboard.putNumber("TurnPID/VelError", getController().getVelocityError());
        SmartDashboard.putNumber("TurnPID/Goal", getController().getGoal().position);
        SmartDashboard.putNumber("TurnPID/Setpoint", getController().getSetpoint().position);
        SmartDashboard.putNumber("TurnPID/Supplied Angle", m_drive.getPitch());
        SmartDashboard.putNumber("TurnPID/Output", getController().calculate(m_drive.getPitch()));
        SmartDashboard.putNumber("TurnPID/Count", count);
        
        if (getController().atGoal()) {
        count++;
        } else {
        count = 0;
        }

        return count > 10;
    }
}
    

