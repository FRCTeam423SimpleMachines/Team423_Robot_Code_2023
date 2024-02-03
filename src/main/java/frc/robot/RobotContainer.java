// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DoNothingAuton;
import frc.robot.commands.DriveAuton;
import frc.robot.commands.DriveAutonLowScore;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.arm.MoveArm1NonPID;
import frc.robot.commands.arm.MoveArm1Pos;
import frc.robot.commands.arm.MoveArm2NonPID;
import frc.robot.commands.arm.MoveArm2Pos;
import frc.robot.commands.arm.MoveArmsNonPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.balanceAuton.BalanceAuton;
import frc.robot.commands.balanceAuton.BalanceAutonLowGoal;
import frc.robot.commands.balanceAuton.BalancePath1;
import frc.robot.commands.balanceAuton.BalancePath2;
import frc.robot.commands.gripper.MoveWristNonPID;
import frc.robot.commands.scoringAuton.ScoreHighBalance;
import frc.robot.commands.scoringAuton.ScoreHighDoNothing;
import frc.robot.commands.scoringAuton.ScoreHighDrive;
import frc.robot.commands.scoringAuton.ScorePathBalance;
import frc.robot.commands.scoringAuton.ScorePathDrive;
import frc.robot.commands.visionAim.TagAlign;
import frc.robot.commands.visionAim.TagShift;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.ResourceBundle.Control;
import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final GripperSubsystem m_GripperSubsystem = new GripperSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

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
    m_chooser.addOption("Score low and drive foward", new DriveAutonLowScore(m_DriveSubsystem, m_ArmSubsystem, DriveDistance.returnController(m_DriveSubsystem)));
    m_chooser.addOption("Charging Station Balance", new BalanceAuton(m_DriveSubsystem, BalancePath1.returnController(m_DriveSubsystem), BalancePath2.returnController(m_DriveSubsystem)));
    m_chooser.addOption("Score High and Balance", new ScoreHighBalance(m_DriveSubsystem, m_ArmSubsystem, m_GripperSubsystem, ScorePathBalance.returnController(m_DriveSubsystem)));
    m_chooser.addOption("Score High and Drive", new ScoreHighDrive(m_DriveSubsystem, m_ArmSubsystem, m_GripperSubsystem, ScorePathDrive.returnController(m_DriveSubsystem)));
    m_chooser.addOption("Score High and Do Nothing", new ScoreHighDoNothing(m_ArmSubsystem, m_GripperSubsystem, ScorePathDrive.returnController(m_DriveSubsystem)));
    m_chooser.addOption("Charging Station Balance with Low Goal", new BalanceAutonLowGoal(m_DriveSubsystem, m_ArmSubsystem, BalancePath1.returnController(m_DriveSubsystem), BalancePath2.returnController(m_DriveSubsystem)));


    // Put the chooser on the dashboard
    CameraServer.startAutomaticCapture();
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
        () -> m_ArmSubsystem.setArmPower(0.5*m_driverController2.getRawAxis(ControlConstants.kRightYAxis),0),
        m_ArmSubsystem
        )
    );

    m_GripperSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_GripperSubsystem.moveWrist(MathUtil.applyDeadband(m_driverController2.getRawAxis(Constants.ControlConstants.kLeftYAxis), 0)), 
        m_GripperSubsystem)
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

    // Drive Controls
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

    
    // Wrist Controls

    /* Test Code
    new JoystickButton(m_driverController, Constants.ControlConstants.kBButton)
    .whileTrue(new RunCommand(
        () -> m_GripperSubsystem.moveWrist(0.5),
        m_GripperSubsystem ));

    new JoystickButton(m_driverController, Constants.ControlConstants.kAButton)
    .whileTrue(new RunCommand(
        () -> m_GripperSubsystem.moveWrist(-0.5),
        m_GripperSubsystem ));
    */

    // Opens and closes gripper
    new JoystickButton(m_driverController2, ControlConstants.kLeftBumber).toggleOnTrue(new InstantCommand(()-> m_GripperSubsystem.activateGripper(), m_GripperSubsystem));
    //new JoystickButton(m_driverController2, ControlConstants.kXButton).toggleOnTrue(new MoveWristNonPID(200, m_GripperSubsystem));
    

    
    
    //Stupid vision buttons
   

    new JoystickButton(m_driverController2, Constants.ControlConstants.kBButton)
    .onTrue(new TagAlign(m_VisionSubsystem, m_DriveSubsystem));

    new JoystickButton(m_driverController2, Constants.ControlConstants.kXButton)
    .onTrue(new TagShift(m_VisionSubsystem, m_DriveSubsystem));

    // Arm Controls
    // Test Code
    /*
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
        .onTrue(new MoveArm1NonPID(75, m_ArmSubsystem));

    new JoystickButton(m_driverController2, ControlConstants.kLeftBumber)
        .onTrue(new MoveArm2NonPID(0, m_ArmSubsystem));
    
    */ /*
    new JoystickButton(m_driverController2, ControlConstants.kYButton)
      .onTrue(new MoveArmsNonPID(ArmConstants.kArm1HighAngle, ArmConstants.kArm2HighAngle, m_ArmSubsystem)
    );
    
    new JoystickButton(m_driverController2, ControlConstants.kRightBumber)
      .onTrue(new ParallelCommandGroup(new MoveWristNonPID(20, m_GripperSubsystem),new MoveArmsNonPID(Constants.ArmConstants.kArm1StartingAngle, Constants.ArmConstants.kArm2StartingAngle, m_ArmSubsystem))
    );

    /*new JoystickButton(m_driverController2, ControlConstants.kBButton)
      .onTrue(new MoveArmsNonPID(ArmConstants.kArm1MiddleAngle, ArmConstants.kArm2MiddleAngle, m_ArmSubsystem)
    );*/
/* 
    new JoystickButton(m_driverController2, ControlConstants.kAButton)
      .onTrue(new ParallelCommandGroup(new MoveWristNonPID(126, m_GripperSubsystem), new MoveArmsNonPID(ArmConstants.kArm1PickupAngle, ArmConstants.kArm2PickupAngle, m_ArmSubsystem))
    );
    new JoystickButton(m_driverController2, ControlConstants.kXButton)
      .onTrue(new MoveArmsNonPID(ArmConstants.kArm1StationAngle, ArmConstants.kArm2StationAngle, m_ArmSubsystem)
    );
    
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
