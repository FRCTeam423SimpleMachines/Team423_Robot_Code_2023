package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmsPos extends CommandBase{

    ArmSubsystem m_armsubsystem;
    double goal1, goal2;

    public MoveArmsPos(double angle1, double angle2, ArmSubsystem arm){
        goal1 = angle1;
        goal2 = angle2;
        arm.setArmPos(angle1, angle2);
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
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentAngle1 = m_armsubsystem.getArm1Angle();
        double currentAngle2 = m_armsubsystem.getArm2Angle();
        if (((currentAngle1 - goal1) > -5) && ((currentAngle1 - goal1) < 5) && ((currentAngle2 - goal2) > -5) && ((currentAngle2 - goal2) < 5)){
            return true;
        } else {
            return false;
        }
    }

}
