// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }



  public static class ControlConstants {
    public static final int kLeftXAxis= 0;
    public static final int kLeftYAxis= 1;
    public static final int kRightXAxis = 4;
    public static final int kRightYAxis = 5;

    public static final int kLeftTrigger = 2;
    public static final int kRightTrigger = 3;

    public static final int kAButton = 1;
    public static final int kBButton = 2;
    public static final int kXButton = 3;
    public static final int kYButton = 4;
    public static final int kLeftBumber = 5;
    public static final int kRightBumber = 6;
    public static final int kBackButton = 7;
    public static final int kStartButton = 8;
    public static final int kLeftStickButton = 9;
    public static final int kRoghtStickButton = 10;
    
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.3; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 1.0; // percent per second (1 = 100%)

    public static final double kDriveBaseWidth = 24; //inches
    public static final double kDriveBaseLength = 30; //inches

    public static final double kDistToWheelCenters = 1.5; //inches

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(kDriveBaseWidth-(kDistToWheelCenters*2));
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(kDriveBaseLength-(kDistToWheelCenters*2));
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFieldOrientOffset = 90;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 12;
    public static final int kRearLeftDrivingCanId = 13;
    

    public static final int kFrontLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 15;
    public static final int kRearRightTurningCanId = 16;
    public static final int kRearLeftTurningCanId = 17;
    

    public static final boolean kGyroReversed = false;

    public static enum Direction {FORWARD, REVERSE, LEFT, RIGHT;}

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class ArmConstants {
    public static final int kArm1CanId = 20;
    public static final int kArm2CanId = 21;
    
    public static final int kArm1EncoderChannel = 0;
    public static final int kArm2EncoderChannel = 1;

    public static final double kArm1MaxVelocity = 5;
    public static final double kArm1MaxAcceleration = 2;

    public static final double kArm2MaxVelocity = 3;
    public static final double kArm2MaxAcceleration = 1;
    
    public static final double kArm1P = .1;
    public static final double kArm1I = 0;
    public static final double kArm1D = 0.4;

    public static final double kArm2P = .1;
    public static final double kArm2I = 0.001;
    public static final double kArm2D = 0.4;

    public static final double kArm1S = 0;
    public static final double kArm1G = 0;
    public static final double kArm1V = 0;
    public static final double kArm1A = 0;

    public static final double kArm2S = 0;
    public static final double kArm2G = 0;
    public static final double kArm2V = 0;
    public static final double kArm2A = 0;

    public static final double kArm1Length = 33; //inches
    public static final double kArm2Length = 26; //inches
    public static final double kArm1VerticalOffset = 13.75; //inches
    public static final double kArm1HorizontalOffset = 18; //inches

    public static final double kArm1MinAngle = 30;
    public static final double kArm1MaxAngle = 120;
    
    public static final double kArm2MinAngle = -58;
    public static final double kArm2MaxAngle = 130;
    
    public static final double kArm1EncoderAngleOffset = 52;
    public static final double kArm2EncoderAngleOffset = -100;

    public static final double kArm1StartingAngle = 108;
    public static final double kArm2StartingAngle = -60;

    public static final double kArm1PickupAngle = 33;
    public static final double kArm2PickupAngle = -10;

    public static final double kArm1LowAngle = 33;
    public static final double kArm2LowAngle = -20;

    public static final double kArm1MiddleAngle = 77;
    public static final double kArm2MiddleAngle = 0;

    public static final double kArm1HighAngle = 44;
    public static final double kArm2HighAngle = 60;

    //public static final double kArm1StationAngle = 100;
    //public static final double kArm2StationAngle = 0;

    public static final double kArm1StationAngle = 110;
    public static final double kArm2StationAngle = -10;
    
    
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriver2ControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


    //Feild distances in meters (described)
    /*
    (all measurements with endpoints on the charging station are calculated for balancing)
    - Edge of community to mid-field (TBx) - 85.13in/2.162302m
    - End-scoring node to directly across from pre-placed mid-field cargo (TBy) - 16in/0.4064m
    - Middle-scoring node to directly across from pre-placed mid-field cargo (TBy) - 6in/0.1524m
    - Scoring node to mid-field cargo (TBx) - 224in/5.6896m
    - Scoring node to charging station (Mx) - 60.69in/1.541526m
    - Inner edge of charging station to mid-field cargo (Mx) - 163.31in/4.148074m
    - Middle scoring node to pre-placed mid-field cargo (Mx) - 222.32in/5.646928m
    - Edge of community to balanced on charging station (Mx) - 51in/1.2954m
    - Middle scoring node to edge of community (charging station) (Mx) - 137.19in/3.484626m
    - Middle scoring node to right across from pre-placed mid-field cargo 2 (My) - 24in/0.6096m
    - Middle scoring node to right across from pre-placed mid-field cargo 3 (My) - 24in/0.6096m
    - Mid-field cargo to charging station - middle (Mx) - 136.13in/3.457702m
    */
    //Feild distances in meters (initialized)
    public static final double kCommunityToMidTBx = 2.16230;
    public static final double kEndNodeToMidCargoTBy = 0.4064;
    public static final double kMidNodeToMidCargoTBy = 0.1524;
    public static final double kNodeToMidCargoTBx = 5.6896;
    public static final double kNodeToChargeMx = 1.541526;
    public static final double kInnerChargeToMidCargoMx = 4.148074;
    public static final double kMidNodeToMidCargoMx = 5.646928;
    public static final double kEdgeCommunityToChargeMx = 1.2954;
    public static final double kMidNodeToCommunityEdgeMx = 3.484626;
    public static final double kMidNodeToMidCargosMy = 0.6096;
    public static final double kMidCargoToChargeM = 3.457702;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class GripperConstants{

    /* 
     * Wrist Constants
     */

     //Assign PWM ID for encoder
     public static final int kWristEncoderPwmID = 2;
    
    //Assign CAN ID of wrist motor
     public static int kWristMotorCanID = 22;

     // Setpoint in degrees of initial position
     public static double kWristInitialSetpoint = 87;
 
     // How many radians of movement of the gripper per rotation of the motor
     public static final double kMotorRotationsToRadians = 2 * Math.PI;
     // Maximum and minimum range of the wrist
     public static final double kWristMaxSetPoint = 3;
     public static final double kWristMinSetPoint = 200;

     public static final double kWristEncoderPositionPIDMinInput = 0; // radians
     public static final double kWristEncoderPositionPIDMaxInput = (2 * Math.PI);

     public static final double kWristEncoderOffset = 0;

             // PID Variables
    public static final double kP = 1; 
    public static final double kI = 0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0; 


    // Invert the encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kWristEncoderInverted = true;

    public static final IdleMode kWristMotorIdleMode = IdleMode.kBrake;

    public static final int kWristMotorCurrentLimit = 40; // amps
 
     /*
      * Gripper constants
      */

    //There are two options for operating solenoids to control pneumatic cylinders, 
    //the CTRE Pneumatics Control Module and the REV Robotics Pneumatics Hub
    //Valid values are "CTREPCM" or "REV"
    public static PneumaticsModuleType kPneumaticsModuleType = PneumaticsModuleType.CTREPCM;
   
    //Assign forward and reverse channels of the solenoid
    public static final int kSolenoidForwardChannel = 0;
    public static final int kSolenoidReverseChannel = 1;

    //Assign forward and reverse solenoid ports to gripper states
    public static DoubleSolenoid.Value kGripperOpen = DoubleSolenoid.Value.kForward;
    public static DoubleSolenoid.Value kGripperClosed = DoubleSolenoid.Value.kReverse;
    
    //Assign initial state of the gripper when created
    public static DoubleSolenoid.Value kGripperInitialState = kGripperOpen;

    //Assign CAN ID of the Pnuematic Control Module.  By default the system sets this 0
    public static final int kSolenoidPCMCanId = 7;



  }

}
