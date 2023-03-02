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

public class VisionSubsystem extends SubsystemBase {

   /**
     * Sets the LED State of the LimeLight
     * @param ledState - 1: force off 2: force blink 3: force on
     */
    public void setLimeLightLED(int ledState) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledState);
  }

  private int pipeline = 0;
//first three to be changed later
  private final double LENGTH = 2.5;
  private final double HEIGHT = 7.458333333;
  private final double LIMELIGHT_HEIGHT = 1.833333;
  public double tv;
  public double tx;
  public double ty;
  public double ta;
  private double ts;
  private double tl;
  private double tshort;
  private double tlong;
  private double thor;
  private double tvert;
  private double distanceToGoal;
  private double angleToGoal;

  private Integer angleOnGoalCount = 0;

  public VisionSubsystem() {
      tv = 0.0;
      tx = 0.0;
      ty = 0.0;
      ta = 0.0;
      tlong = 0.0;
      tshort = 0.0;
      distanceToGoal = 0.0;
      angleToGoal = 0.0;
      angleOnGoalCount = 0;
  }

  /**
   * This is the method for pulling values from LimeLight
   * This is called in the Robot Periodic Method
   */
  public void run() {
      tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
      tlong = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
      tshort = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
      calculateDistance();
      calculateAngle();
      logToDashBoard();
  }

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    for (int port = 5800; port <= 5805; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

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
    //Used for estimating distance of target (april tag or lower relfective tape target) (measurements to be replaced at a later date)
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    double targetFloorDistance;

    if (pipeline == 0) {
      targetFloorDistance = 17;
    }
    else if (pipeline == 1){ 
      if (0<ty<26)
        targetFloorDistance = 22.75;
      else {
        targetFloorDistance = 46;
      }
    }

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = targetFloorDistance;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    
    return distanceFromLimelightToGoalInches;
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
  public double inchesToMeters (double inch){
    return inch *= 0.0254;
  }
}
