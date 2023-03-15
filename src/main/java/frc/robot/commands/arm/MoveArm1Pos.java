package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm1Pos extends ProfiledPIDCommand{

 
    ArmSubsystem m_armsubsystem;
    private int count = 0;

    public MoveArm1Pos(double angle, ArmSubsystem arm1){

        super(
            new ProfiledPIDController(
                ArmConstants.kArm1P,
                ArmConstants.kArm1I,
                ArmConstants.kArm1D,
                new TrapezoidProfile.Constraints(
                    ArmConstants.kArm1MaxVelocity,
                    ArmConstants.kArm1MaxAcceleration
                )
            ),
            () -> arm1.getArm1Angle(),
            angle,
            (output, setpoint) -> arm1.setArm1Power(output),
            arm1
        );

        m_armsubsystem = arm1;
        getController().enableContinuousInput(-180, 180);

        getController().setTolerance(5.0);


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    

    // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        
        super.end(interrupted);
        //m_armsubsystem.setArm1Pos(0.0);
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
