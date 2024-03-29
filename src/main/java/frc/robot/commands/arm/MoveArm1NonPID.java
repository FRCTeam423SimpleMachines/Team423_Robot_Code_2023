package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm1NonPID extends CommandBase{
 
    ArmSubsystem m_armsubsystem;
    double goal;
    double tolerance = 3;

    public MoveArm1NonPID(double angle, ArmSubsystem arm){    
        goal = angle;
        m_armsubsystem = arm;
        arm.setArm1Pos(angle);
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_armsubsystem.getArm1Angle() > goal + tolerance){
            m_armsubsystem.setArm1Power(-0.8);
        } else if (m_armsubsystem.getArm2Angle() < goal - tolerance){
            m_armsubsystem.setArm1Power(0.8);
        } else {
            m_armsubsystem.setArm1Power(0);
        }
    }

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        m_armsubsystem.setArm1Power(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentAngle = m_armsubsystem.getArm1Angle();
        if (((currentAngle - goal) > -tolerance) && ((currentAngle - goal) < tolerance)){
            return true;
        } else {
            return false;
        }
    }

}
