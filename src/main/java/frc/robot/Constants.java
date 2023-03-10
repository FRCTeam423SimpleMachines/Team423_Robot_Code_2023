// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

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

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class ArmConstants {
    public static final int kArm1CanId = 20;
    public static final int kArm2CanId = 21;
    
    public static final int kArm1EncoderChannel = 0;
    public static final int kArm2EncoderChannel = 1;

    public static final double kArm1MaxVelocity = 1.75;
    public static final double kArm1MaxAcceleration = 0.75;

    public static final double kArm2MaxVelocity = 1.75;
    public static final double kArm2MaxAcceleration = 0.75;
    
    public static final double kArm1P = 1;
    public static final double kArm1I = 0;
    public static final double kArm1D = 0;

    public static final double kArm2P = 1;
    public static final double kArm2I = 0;
    public static final double kArm2D = 0;

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
    
    public static final double kArm1EncoderAngleOffset = 50;
    public static final double kArm2EncoderAngleOffset = -275;

    public static final double kArm1StartingAngle = 110;
    public static final double kArm2StartingAngle = -58;
    
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
}
