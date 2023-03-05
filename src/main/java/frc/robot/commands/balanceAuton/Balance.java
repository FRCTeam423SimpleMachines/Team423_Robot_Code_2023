package frc.robot.commands.balanceAuton;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;


public class Balance extends CommandBase{

    DriveSubsystem m_DriveSubsystem;
    int counter;
    double pitch;
    
    public Balance (DriveSubsystem DriveSubsystem){
        m_DriveSubsystem = DriveSubsystem;
        counter = 0;
        addRequirements(m_DriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pitch = m_DriveSubsystem.getPitch();

        if (pitch < -3 ){
            counter = 0;
            m_DriveSubsystem.drive(0.075, 0, 0, true, false);
        } else if (pitch > 4 ){
            counter = 0;
            m_DriveSubsystem.drive(-0.075, 0, 0, true, false);
        } else {
            counter ++;
            m_DriveSubsystem.drive(0, 0, 0, true, false);
        }
    }

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (counter >= 5){
            return true;
        } else {
            return false;
        }
    }
}
