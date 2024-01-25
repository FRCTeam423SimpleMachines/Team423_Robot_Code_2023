package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;

public class MoveWristNonPID extends Command{
 
    GripperSubsystem m_grippersubsystem;
    double goal;
    double tolerance = 3;

    public MoveWristNonPID(double angle, GripperSubsystem gripper){    
        goal = angle;
        m_grippersubsystem = gripper;
        addRequirements(gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_grippersubsystem.getWristAngle() > goal + tolerance){
            m_grippersubsystem.moveWrist(-0.8);
        } else if (m_grippersubsystem.getWristAngle() < goal - tolerance){
            m_grippersubsystem.moveWrist(0.8);
        } else {
            m_grippersubsystem.moveWrist(0);
        }
    }

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        m_grippersubsystem.moveWrist(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentAngle = m_grippersubsystem.getWristAngle();
        if (((currentAngle - goal) > -tolerance) && ((currentAngle - goal) < tolerance)){
            return true;
        } else {
            return false;
        }
    }

}
