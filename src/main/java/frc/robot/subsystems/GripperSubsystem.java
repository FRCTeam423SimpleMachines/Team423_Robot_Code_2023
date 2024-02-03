// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
    
  private DoubleSolenoid m_solenoid;
  private DutyCycleEncoder m_wristEncoder = new DutyCycleEncoder(GripperConstants.kWristEncoderPwmID);
  private CANSparkMax m_wristMotor = new CANSparkMax(GripperConstants.kWristMotorCanID, MotorType.kBrushless);
  private ProfiledPIDCommand m_pidController;
  private double offset;
  GenericEntry wristPos = Shuffleboard.getTab("Gripper").add("Gripper Pos",m_wristEncoder.getDistance()).getEntry();
  GenericEntry wristConnected = Shuffleboard.getTab("Gripper").add("Gripper Connected",m_wristEncoder.isConnected()).getEntry();
  
  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
  
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_wristMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    
    m_wristEncoder.setDistancePerRotation(360);
    // Apply position conversion factors for the wrist encoder. The
    // native units for position are rotations but we want radians
    //m_wristEncoder.setPositionConversionFactor(GripperConstants.kMotorRotationsToRadians);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    //m_wristEncoder.setInverted(GripperConstants.kWristEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    //m_pidController.setPositionPIDWrappingEnabled(true);
    //m_pidController.setPositionPIDWrappingMinInput(GripperConstants.kWristEncoderPositionPIDMinInput);
    //m_pidController.setPositionPIDWrappingMaxInput(GripperConstants.kWristEncoderPositionPIDMaxInput);

    // Set PID parameters
    /*m_pidController.setI(GripperConstants.kI);
    m_pidController.setD(GripperConstants.kD);
    m_pidController.setIZone(GripperConstants.kIz);
    m_pidController.setFF(GripperConstants.kFF);
    m_pidController.setOutputRange(GripperConstants.kWristMinSetPoint, GripperConstants.kWristMaxSetPoint);
    */
    m_wristMotor.setIdleMode(GripperConstants.kWristMotorIdleMode);
    m_wristMotor.setSmartCurrentLimit(GripperConstants.kWristMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    //m_wristMotor.burnFlash();
    
    // Move wrist to initial position
    //this.setWristAngleRadians(GripperConstants.kWristInitialSetpoint);   

    // Create a new solenoid for controlling the pneumatic
    m_solenoid = new DoubleSolenoid(GripperConstants.kSolenoidPCMCanId, GripperConstants.kPneumaticsModuleType, 
    GripperConstants.kSolenoidForwardChannel, GripperConstants.kSolenoidReverseChannel);
  
    // Initialize the solenoid so the gripper knows where to start.  
    m_solenoid.set(GripperConstants.kGripperInitialState);

    if (m_wristEncoder.getDistance() > 270 ){
      offset = -360 + GripperConstants.kWristEncoderOffset;
    } else {
      offset = GripperConstants.kWristEncoderOffset;
    }
  }
  
 /* 
 Wrist Code 
 */

  public void setWristAngleDegrees(double angle)
  {
    setWristAngleRadians(Math.toRadians(angle));
    
  }


  public void setWristAngleRadians(double angle)
  {
    // Calculates the output of the PID algorithm based on the sensor reading
    // and sends it to a motor
    if (setPointValid(angle))
    {
    //  m_pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }
  }

  public void moveWrist(double speed)
  {
    
    if(getWristAngle() <= GripperConstants.kWristMaxSetPoint && speed < 0) {
      speed = 0;
    }
    if(getWristAngle() >= GripperConstants.kWristMinSetPoint && speed > 0) {
      speed = 0;
    }
    
    if (Math.abs(speed) > 0.01)
      m_wristMotor.set(speed);
    else
      m_wristMotor.set(0);
  }

  public boolean setPointValid(double setpoint)
  {
    return setpoint > GripperConstants.kWristMinSetPoint && setpoint < GripperConstants.kWristMaxSetPoint;
  }

  public double getWristAngle(){
    return (m_wristEncoder.getDistance()) + offset;
  }

  
/* 
 * GRIPPER CODE
 */

  public void activateGripper()
  {
    m_solenoid.toggle();
  }
  
  public void openGripper()
  {
    m_solenoid.set(GripperConstants.kGripperOpen);
  }

  public void closeGripper()
  {
    m_solenoid.set(GripperConstants.kGripperClosed);
  }

  @Override
  public void periodic() {
    wristPos.setDouble(getWristAngle());
    wristConnected.setBoolean(m_wristEncoder.isConnected());
  }
}
