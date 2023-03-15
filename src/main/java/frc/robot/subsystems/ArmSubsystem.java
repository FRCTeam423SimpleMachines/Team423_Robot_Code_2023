// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_arm1 = new WPI_TalonFX(ArmConstants.kArm1CanId);
  private final CANSparkMax m_arm2 = new CANSparkMax(ArmConstants.kArm2CanId, MotorType.kBrushless);

  private DutyCycleEncoder m_arm1Encoder = new DutyCycleEncoder(ArmConstants.kArm1EncoderChannel);
  private DutyCycleEncoder m_arm2Encoder = new DutyCycleEncoder(ArmConstants.kArm2EncoderChannel);

  private final TrapezoidProfile.Constraints m_arm1Constraints = new TrapezoidProfile.Constraints(ArmConstants.kArm1MaxVelocity, ArmConstants.kArm1MaxAcceleration);
  private final TrapezoidProfile.Constraints m_arm2Constraints = new TrapezoidProfile.Constraints(ArmConstants.kArm2MaxVelocity, ArmConstants.kArm2MaxAcceleration);
  
  private final ProfiledPIDController m_arm1Controller = new ProfiledPIDController(ArmConstants.kArm1P, ArmConstants.kArm1I, ArmConstants.kArm1D, m_arm1Constraints);
  private final ProfiledPIDController m_arm2Controller = new ProfiledPIDController(ArmConstants.kArm2P, ArmConstants.kArm2I, ArmConstants.kArm2D, m_arm2Constraints);

  private ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private GenericEntry arm1Dist = armTab.add("Arm 1 Distance", m_arm1Encoder.getDistance()).getEntry();
  private GenericEntry arm1Connect = armTab.add("Arm 1 Connected", m_arm1Encoder.isConnected()).getEntry();
  private GenericEntry arm2Dist = armTab.add("Arm 2 Distance", m_arm2Encoder.getDistance()).getEntry();
  private GenericEntry arm2Connect = armTab.add("Arm 2 Connected", m_arm2Encoder.isConnected()).getEntry();

  private boolean armSfty = true;
  GenericEntry armSafety = Shuffleboard.getTab("Safeties").add("Arm Safety", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

  private double arm1EncoderOffset;
  private double arm2EncoderOffset;

  //ArmFeedforward arm1Feedforward = new ArmFeedforward(ArmConstansts.kArm1S,ArmConstansts.kArm1G,ArmConstansts.kArm1V,ArmConstansts.kArm1A);
  //ArmFeedforward arm2Feedforward = new ArmFeedforward(ArmConstansts.kArm2S,ArmConstansts.kArm2G,ArmConstansts.kArm2V,ArmConstansts.kArm2A);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    m_arm1Encoder.setDistancePerRotation(360);
    m_arm2Encoder.setDistancePerRotation(360);


    if (m_arm1Encoder.getDistance() > 270){
      arm1EncoderOffset = ArmConstants.kArm1EncoderAngleOffset - 360;
    } else {
      arm1EncoderOffset = ArmConstants.kArm1EncoderAngleOffset;
    }

    if (m_arm2Encoder.getDistance() > 200){
      arm2EncoderOffset = ArmConstants.kArm2EncoderAngleOffset - 360;
    } else {
      arm2EncoderOffset = ArmConstants.kArm2EncoderAngleOffset;
    }    

    m_arm1Controller.setGoal(ArmConstants.kArm1StartingAngle);
    m_arm2Controller.setGoal(ArmConstants.kArm2StartingAngle);

    //m_arm1Encoder.setPositionOffset(ArmConstants.kArm1EncoderAngleOffset);
    //m_arm2Encoder.setPositionOffset(ArmConstants.kArm2EncoderAngleOffset);

    armTab = Shuffleboard.getTab("Arm");
    arm1Dist = armTab.add("Arm 1 Distance", m_arm1Encoder.getDistance()).getEntry();
    arm1Connect = armTab.add("Arm 1 Connected", m_arm1Encoder.isConnected()).getEntry();
    arm2Dist = armTab.add("Arm 2 Distance", m_arm2Encoder.getDistance()).getEntry();
    arm2Connect = armTab.add("Arm 2 Connected", m_arm2Encoder.isConnected()).getEntry();
    
  }

  public void setArm1Power(double pow) {
    if (armSfty) {
      if (Math.abs(pow) <= .4)
        m_arm1.set(pow);
      else
        if (pow > .4)
          m_arm1.set(.4);
        if (pow < -.4)
          m_arm1.set(-0.4);
    } else {
      m_arm1.set(0.0);
    }

  }

  public void setArm2Power(double pow) {
    pow = -pow;
    if (armSfty) {
      if (Math.abs(pow) <= .3)
        m_arm2.set(pow);
      else
        if (pow > .3)
          m_arm2.set(.3);
        if (pow < -.3)
          m_arm2.set(-0.3);
    } else {
      m_arm2.set(0.0);
    }
  }

  public void updateArm1Pos(){
    setArm1Power(m_arm1Controller.calculate(getArm1Angle()));    
  }

  public void updateArm2Pos(){
    setArm2Power(m_arm2Controller.calculate(-getArm2Angle()));
  }

  public void updateArmsPos() {
    updateArm1Pos();
    updateArm2Pos();
  }
  
  public void setArm1Pos(double pos) {
    if (pos < ArmConstants.kArm1MinAngle) {
      pos = ArmConstants.kArm1MinAngle;
    }

    if (pos > ArmConstants.kArm1MaxAngle) {
      pos = ArmConstants.kArm1MaxAngle;
    }

    m_arm1Controller.setGoal(pos);
  }

  public void setArm2Pos(double pos) {
    if (pos < ArmConstants.kArm2MinAngle) {
      pos = ArmConstants.kArm2MinAngle;
    }

    if (pos > ArmConstants.kArm2MaxAngle) {
      pos = ArmConstants.kArm2MaxAngle;
    }

    m_arm2Controller.setGoal(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //updateArmsPos();
    
    //SmartDashboard.putNumber("Arm/Arm 1 Position", m_arm1Encoder.getDistance());
    //SmartDashboard.putData("Arm 1 Encoder", m_arm1Encoder);
    arm1Dist.setDouble(m_arm1Encoder.getDistance());
    arm1Connect.setBoolean(m_arm1Encoder.isConnected());
    arm2Dist.setDouble(m_arm2Encoder.getDistance());
    arm2Connect.setBoolean(m_arm2Encoder.isConnected());
    Shuffleboard.update();

    armSfty = armSafety.getBoolean(true);
    
    logToDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void logToDashboard() {
    arm1Dist.setDouble(m_arm1Encoder.getDistance());
    arm1Connect.setBoolean(m_arm1Encoder.isConnected());
    arm2Dist.setDouble(m_arm2Encoder.getDistance());
    arm2Connect.setBoolean(m_arm2Encoder.isConnected());
  }
    
  public void setArmPower(double i, double j) {
    setArm1Power(i);
    setArm2Power(j);
  }

  public void setArmPos(double i, double j){
    setArm1Pos(i);
    setArm2Pos(j);
  }

  public double getArm1Angle(){
    return m_arm1Encoder.getDistance() + arm1EncoderOffset;
  }

  public double getArm2Angle(){
    return m_arm2Encoder.getDistance() + arm2EncoderOffset;
  }



  public double getArm1RawAngle(){
    return m_arm1Encoder.getDistance();
  }

  public double getArm2RawAngle(){
    return m_arm2Encoder.getDistance();
  }
}