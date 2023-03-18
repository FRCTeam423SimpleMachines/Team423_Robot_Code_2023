package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmsNonPID extends CommandBase{

    ArmSubsystem m_armsubsystem;
    double goal1, goal2;
    double tolerance = 3;

    public MoveArmsNonPID(double angle1, double angle2, ArmSubsystem arm){
        goal1 = angle1;
        goal2 = angle2;
        m_armsubsystem = arm;
        arm.setArmPos(angle1, angle2);
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_armsubsystem.getArm1Angle() > goal1 + tolerance){
            m_armsubsystem.setArm1Power(-0.8);
        } else if (m_armsubsystem.getArm2Angle() < goal1 - tolerance){
            m_armsubsystem.setArm1Power(0.8);
        } else {
            m_armsubsystem.setArm1Power(0);
        }
        

        if (m_armsubsystem.getArm2Angle() > goal2 + tolerance){
            m_armsubsystem.setArm2Power(-0.2);
        } else if (m_armsubsystem.getArm2Angle() < goal2 - tolerance){
            m_armsubsystem.setArm2Power(0.2);
        } else {
            m_armsubsystem.setArm2Power(0);
        }

    }

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        m_armsubsystem.setArm1Power(0);
        m_armsubsystem.setArm2Power(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentAngle1 = m_armsubsystem.getArm1Angle();
        double currentAngle2 = m_armsubsystem.getArm2Angle();
        if (((currentAngle1 - goal1) > -tolerance) && ((currentAngle1 - goal1) < tolerance) && ((currentAngle2 - goal2) > -tolerance) && ((currentAngle2 - goal2) < tolerance)){
            return true;
        } else {
            return false;
        }
    }

}
