// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//To enable tethered limelight communication while tethered to the robot (before competition - day of)
import edu.wpi.first.wpiutil.net.PortForwarder;


@Override
public void robotInit()
{
    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    for (int port = 5800; port <= 5805; port++) {
        PortForwarder.add(port, "limelight.local", port);
    }
}

public class VisionSubsystem extends SubsystemBase {

NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

cameraMatrix = cv::Matx33d(
                        772.53876202, 0., 479.132337442,
                        0., 769.052151477, 359.143001808,
                        0., 0., 1.0);
distortionCoefficient =  std::vector<double> {
                        2.9684613693070039e-01, -1.4380252254747885e+00,-2.2098421479494509e-03,
                        -3.3894563533907176e-03, 2.5344430354806740e+00};

focalLength = 2.9272781257541; //mm

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);

private int pipeline = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getDistance (){
    if (pipeline == 0) {
      setDistance(/*distance between center of target and ground */);
    }
    else { 
      setDistance(/*distance between center of target and ground */);
    }
  }

  public double setDistance (double targetFloorDistance) {
    //Used for estimating distance of target (april tag or lower relfective tape target) (measurements to be replaced at a later date)
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = targetFloorDistance;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
  }

//code to auto-adjust till desired angle is reached
  /*public double adjustAngle (){
    float Kp = -0.1f;
    float min_command = 0.05f;

    std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
    float tx = table->GetNumber("tx");
            float heading_error = -tx;
            float steering_adjust = 0.0f;
    if (Math.abs(heading_error) > 1.0)
            {
                    if (heading_error < 0)
                    {
                            steering_adjust = Kp*heading_error + min_command;
                    }
                    else
                    {
                            steering_adjust = Kp*heading_error - min_command;
                    }
            }
            left_command += steering_adjust;
            right_command -= steering_adjust;
  }*/
//to switch between pipelines - called with gamepad input
  public void setPipeline(int i){
   pipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(i);
    return pipeline;
  }
}
