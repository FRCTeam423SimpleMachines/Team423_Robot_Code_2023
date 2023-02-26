// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstansts;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_arm1 = new WPI_TalonFX(ArmConstansts.kArm1CanId);
  private final CANSparkMax m_arm2 = new CANSparkMax(ArmConstansts.kArm2CanId, MotorType.kBrushless);

  private DutyCycleEncoder m_arm1Encoder = new DutyCycleEncoder(ArmConstansts.kArm1EncoderChannel);
  private DutyCycleEncoder m_arm2Encoder = new DutyCycleEncoder(ArmConstansts.kArm2EncoderChannel);

  private double arm1Position;
  private double arm2Position;
  
  PIDController arm1Controller = new PIDController(ArmConstansts.kArm1P, ArmConstansts.kArm1I, ArmConstansts.kArm1D);
  PIDController arm2Controller = new PIDController(ArmConstansts.kArm2P, ArmConstansts.kArm2I, ArmConstansts.kArm2D);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

  }

  public void setArm1Power(double pow) {
    m_arm1.set(pow);
  }

  public void setArm2Power(double pow) {
    m_arm2.set(pow);
  }

  public void updateArmsPos() {
    m_arm1.set(arm1Controller.calculate(m_arm1Encoder.getDistance(), arm1Position));
    m_arm2.set(arm2Controller.calculate(m_arm1Encoder.getDistance(), arm2Position));
  }
  
  public void setArm1Pos(double pos) {
    arm1Position = pos;
  }

  public void setArm2Pos(double pos) {
    arm2Position = pos;
  }

  public double getArm1Pos() {
    return m_arm1Encoder.getAbsolutePosition();
  }

  public double getArm2Pos() {
    return m_arm2Encoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateArmsPos();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
