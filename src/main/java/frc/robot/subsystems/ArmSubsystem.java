// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstansts;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_arm1 = new WPI_TalonFX(ArmConstansts.kArm1CanId);
  //private final CANSparkMax m_arm2 = new CANSparkMax(ArmConstansts.kArm2CanId, MotorType.kBrushless);

  private DutyCycleEncoder m_arm1Encoder = new DutyCycleEncoder(ArmConstansts.kArm1EncoderChannel);
  private DutyCycleEncoder m_arm2Encoder = new DutyCycleEncoder(ArmConstansts.kArm2EncoderChannel);

  private final TrapezoidProfile.Constraints m_arm1Constraints = new TrapezoidProfile.Constraints(ArmConstansts.kArm1MaxVelocity, ArmConstansts.kArm1MaxAcceleration);
  private final TrapezoidProfile.Constraints m_arm2Constraints = new TrapezoidProfile.Constraints(ArmConstansts.kArm2MaxVelocity, ArmConstansts.kArm2MaxAcceleration);
  
  private final ProfiledPIDController m_arm1Controller = new ProfiledPIDController(ArmConstansts.kArm1P, ArmConstansts.kArm1I, ArmConstansts.kArm1D, m_arm1Constraints);
  private final ProfiledPIDController m_arm2Controller = new ProfiledPIDController(ArmConstansts.kArm2P, ArmConstansts.kArm2I, ArmConstansts.kArm2D, m_arm2Constraints);

  private ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private GenericEntry arm1Dist = armTab.add("Arm 1 Distance", m_arm1Encoder.getDistance()).getEntry();
  private GenericEntry arm1Connect = armTab.add("Arm 1 Connected", m_arm1Encoder.isConnected()).getEntry();
  private GenericEntry arm2Dist = armTab.add("Arm 2 Distance", m_arm2Encoder.getDistance()).getEntry();
  private GenericEntry arm2Connect = armTab.add("Arm 2 Connected", m_arm2Encoder.isConnected()).getEntry();

  //ArmFeedforward arm1Feedforward = new ArmFeedforward(ArmConstansts.kArm1S,ArmConstansts.kArm1G,ArmConstansts.kArm1V,ArmConstansts.kArm1A);
  //ArmFeedforward arm2Feedforward = new ArmFeedforward(ArmConstansts.kArm2S,ArmConstansts.kArm2G,ArmConstansts.kArm2V,ArmConstansts.kArm2A);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    m_arm1Encoder.setDistancePerRotation(360);
    m_arm2Encoder.setDistancePerRotation(360);

    //Shuffleboard.getTab("Arm Subsystem").add("Arm 1 Encoder", m_arm1Encoder);
    
  }

  public void setArm1Power(double pow) {
    m_arm1.set(pow);
  }

  public void setArm2Power(double pow) {
    //m_arm2.set(pow);
  }

  public void updateArmsPos() {
    //m_arm1.set(m_arm1Controller.calculate(m_arm1Encoder.getDistance()));
    //m_arm2.set(m_arm2Controller.calculate(m_arm1Encoder.getDistance()));
  }
  
  public void setArm1Pos(double pos) {
    m_arm1Controller.setGoal(pos);
  }

  public void setArm2Pos(double pos) {
    m_arm1Controller.setGoal(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateArmsPos();
    
    //SmartDashboard.putNumber("Arm/Arm 1 Position", m_arm1Encoder.getDistance());
    //SmartDashboard.putData("Arm 1 Encoder", m_arm1Encoder);
    arm1Dist.setDouble(m_arm1Encoder.getDistance());
    arm1Connect.setBoolean(m_arm1Encoder.isConnected());
    arm2Dist.setDouble(m_arm2Encoder.getDistance());
    arm2Connect.setBoolean(m_arm2Encoder.isConnected());
    Shuffleboard.update();
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
