// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
    
  private DoubleSolenoid m_solenoid;
  private DutyCycleEncoder m_wristEncoder;
  private CANSparkMax m_wristMotor;
  private ProfiledPIDCommand m_pidController;

  private boolean GrpprSfty = true;

  GenericEntry GripperSafety = Shuffleboard.getTab("Safeties").add("Drive Safety", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  
  /** Creates a new GripperSubsystem. */
  public GripperSubsystem() {
    
    m_wristMotor = new CANSparkMax(GripperConstants.kWristMotorCanID, GripperConstants.kWristMotorType);
    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_wristEncoder = new DutyCycleEncoder(GripperConstants.kWristEncoderPwmID);
    
    //m_wristMotor.setIdleMode(GripperConstants.kWristMotorIdleMode);
    //m_wristMotor.setSmartCurrentLimit(GripperConstants.kWristMotorCurrentLimit);
   
    // Create a new solenoid for controlling the pneumatic
    m_solenoid = new DoubleSolenoid(GripperConstants.kSolenoidPCMCanId, GripperConstants.kPneumaticsModuleType, 
    GripperConstants.kSolenoidForwardChannel, GripperConstants.kSolenoidReverseChannel);
  
    // Initialize the solenoid so the gripper knows where to start.  
    m_solenoid.set(GripperConstants.kGripperInitialState);
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
    if(GrpprSfty) {
      if (Math.abs(speed) > 0.01)
        m_wristMotor.set(speed);
      else
        m_wristMotor.set(0);
    }
  }

  public boolean setPointValid(double setpoint)
  {
    return setpoint > GripperConstants.kWristMinSetPoint && setpoint < GripperConstants.kWristMaxSetPoint;
  }

  
/* 
 * GRIPPER CODE
 */

  public void activateGripper()
  {
    if(GrpprSfty) {
      m_solenoid.toggle();
    }
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
    GrpprSfty = GripperSafety.getBoolean(true);
  }
}
