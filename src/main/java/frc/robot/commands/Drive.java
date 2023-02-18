package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends CommandBase{

    DriveSubsystem m_drive;
    double xSpeed;
    double ySpeed;
    double speed;
    double rot;
    boolean fieldRelative;

    public Drive(double x, double y, double r, Boolean fieldR, DriveSubsystem drive, double spd) {
        m_drive = drive;
        xSpeed = x;
        ySpeed = y;
        speed = spd;
        rot = r;
        fieldRelative = fieldR;

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.drive(xSpeed*speed , ySpeed*speed , rot*speed ,fieldRelative);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
