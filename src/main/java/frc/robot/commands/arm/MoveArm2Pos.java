package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm2Pos extends ProfiledPIDCommand{

  
    ArmSubsystem m_armsubsystem;
    private int count = 0;

    public MoveArm2Pos(double angle, ArmSubsystem arm2){

        super(
            new ProfiledPIDController(
                ArmConstants.kArm2P,
                ArmConstants.kArm2I,
                ArmConstants.kArm2D,
                new TrapezoidProfile.Constraints(
                    ArmConstants.kArm2MaxVelocity,
                    ArmConstants.kArm2MaxAcceleration
                )
            ),
            () -> arm2.getArm2Angle(),
            angle,
            (output, setpoint) -> arm2.setArm2Power(output),
            arm2
        );

        m_armsubsystem = arm2;
        getController().enableContinuousInput(-180, 180);

        getController().setTolerance(5.0);


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        
        super.end(interrupted);
        //m_armsubsystem.setArm2Pos(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (getController().atGoal()) {
            count++;
          } else {
            count = 0;
          }
      
        return count > 10;
    }

}
