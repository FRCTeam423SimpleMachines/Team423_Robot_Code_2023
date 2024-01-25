package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm1Pos extends Command{
 
    ArmSubsystem m_armsubsystem;
    double goal;

    public MoveArm1Pos(double angle, ArmSubsystem arm){    
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
        m_armsubsystem.updateArmsPos();
    }

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        m_armsubsystem.setArm1Power(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentAngle = m_armsubsystem.getArm1Angle();
        if (((currentAngle - goal) > -5) && ((currentAngle - goal) < 5)){
            return true;
        } else {
            return false;
        }
    }

}
