// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DoNothingAuton;
import frc.robot.commands.DriveAuton;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.arm.MoveArm1Pos;
import frc.robot.commands.arm.MoveArm2Pos;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.balanceAuton.BalanceAuton;
import frc.robot.commands.balanceAuton.BalanceAutonLowGoal;
import frc.robot.commands.balanceAuton.BalancePath1;
import frc.robot.commands.balanceAuton.BalancePath2;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ResourceBundle.Control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_driverController2 = new Joystick(OIConstants.kDriverControllerPort+1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
    // Configure the trigger bindings
    configureBindings();

    m_chooser.setDefaultOption("Do Nothing", new DoNothingAuton(m_DriveSubsystem));
    m_chooser.addOption("Drive forward 3m", new DriveAuton(m_DriveSubsystem, DriveDistance.returnController(m_DriveSubsystem)));
    m_chooser.addOption("Charging Station Balance", new BalanceAuton(m_DriveSubsystem, BalancePath1.returnController(m_DriveSubsystem), BalancePath2.returnController(m_DriveSubsystem)));
    //m_chooser.addOption("Charging Station Balance with Low Goal", new BalanceAutonLowGoal(m_DriveSubsystem, m_ArmSubsystem, BalancePath1.returnController(m_DriveSubsystem), BalancePath2.returnController(m_DriveSubsystem)));


    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    m_DriveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_DriveSubsystem.drive(
          MathUtil.applyDeadband(-0.5*squareInput(m_driverController.getRawAxis(Constants.ControlConstants.kLeftYAxis)) , 0.3),
          MathUtil.applyDeadband(-0.5*squareInput(m_driverController.getRawAxis(Constants.ControlConstants.kLeftXAxis)) , 0.3),
          MathUtil.applyDeadband(-0.5*squareInput(m_driverController.getRawAxis(Constants.ControlConstants.kRightXAxis)), 0.3),
          true, true, true), m_DriveSubsystem));
    
    m_ArmSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_ArmSubsystem.setArmPower(0,0),
        m_ArmSubsystem
        )
    );
    
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new JoystickButton(m_driverController, Constants.ControlConstants.kXButton)
        .whileTrue(new RunCommand(
            () -> m_DriveSubsystem.setX(),
            m_DriveSubsystem));

    new JoystickButton(m_driverController, Constants.ControlConstants.kYButton)
        .whileTrue(new RunCommand(
            () -> m_DriveSubsystem.resetGyro(),
            m_DriveSubsystem ));

    new JoystickButton(m_driverController, Constants.ControlConstants.kRightBumber)
        .whileTrue(new RunCommand(
          () -> m_DriveSubsystem.drive(
            MathUtil.applyDeadband(-squareInput(m_driverController.getRawAxis(Constants.ControlConstants.kLeftYAxis)) , 0.3),
            MathUtil.applyDeadband(-squareInput(m_driverController.getRawAxis(Constants.ControlConstants.kLeftXAxis)) , 0.3),
            MathUtil.applyDeadband(-squareInput(m_driverController.getRawAxis(Constants.ControlConstants.kRightXAxis)), 0.3),
            true, false, true), m_DriveSubsystem));
    
    new JoystickButton(m_driverController2, ControlConstants.kAButton)
        .whileTrue(new RunCommand(
          () -> m_ArmSubsystem.setArm2Power(-0.2), m_ArmSubsystem));

    new JoystickButton(m_driverController2, ControlConstants.kXButton)
          .whileTrue(new RunCommand(
            () -> m_ArmSubsystem.setArm2Power(0.2), m_ArmSubsystem)); 

    new JoystickButton(m_driverController2, ControlConstants.kYButton)
        .whileTrue(new RunCommand(
          () -> m_ArmSubsystem.setArm1Power(0.4), m_ArmSubsystem));

    new JoystickButton(m_driverController2, ControlConstants.kBButton)
        .whileTrue(new RunCommand(
          () -> m_ArmSubsystem.setArm1Power(-0.4), m_ArmSubsystem));

    new JoystickButton(m_driverController2, ControlConstants.kRightBumber)
        .onTrue(new MoveArm1Pos(75, m_ArmSubsystem));

    new JoystickButton(m_driverController2, ControlConstants.kLeftBumber)
        .onTrue(new MoveArm2Pos(0, m_ArmSubsystem));
    
    

/* 
    new JoystickButton(m_driverController, Constants.ControlConstants.kLeftBumber)
        .whileTrue(new Drive(
          MathUtil.applyDeadband(!m_driverController.getRawButton(Constants.ControlConstants.kRightBumber) ? -m_driverController.getRawAxis(Constants.ControlConstants.kLeftXAxis) : 0.0, 0.06),
          MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.ControlConstants.kRightTrigger) < 0.6 ? -m_driverController.getRawAxis(Constants.ControlConstants.kLeftYAxis) : 0.0, 0.06),
          MathUtil.applyDeadband(-m_driverController.getRawAxis(Constants.ControlConstants.kRightXAxis), 0.06),
          true, m_DriveSubsystem, 0.5));
*/
            
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public double squareInput(double x){
    if (x > 0){
      return Math.pow(x, 2);
    } else {
      return -Math.pow(x,2);
    }
  }
}
