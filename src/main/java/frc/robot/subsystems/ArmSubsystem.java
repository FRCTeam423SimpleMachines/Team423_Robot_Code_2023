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

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstansts;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_arm1 = new WPI_TalonFX(ArmConstansts.kArm1CanId);
  private final CANSparkMax m_arm2 = new CANSparkMax(ArmConstansts.kArm2CanId, MotorType.kBrushless);

  private AbsoluteEncoder m_Arm2Encoder = m_arm2.getAbsoluteEncoder(Type.kDutyCycle);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_arm1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  public void setArm1Power(double pow) {
    m_arm1.set(pow);
  }

  public void setArm2Power(double pow) {
    m_arm2.set(pow);
  }

  public double getArm1Pos() {
    return m_arm1.getSelectedSensorPosition();
  }

  public double getArm2Pos() {
    return m_Arm2Encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
