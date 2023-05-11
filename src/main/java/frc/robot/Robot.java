// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  NetworkTable table;
  boolean foundObject;
  PIDController angController;

  @Override
  public void robotInit() {
    foundObject = false;
    angController = new PIDController(0.01, 0, 0);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void teleopPeriodic() {
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    if (x != 0) {
      foundObject = true;
    } else {
      foundObject = false;
    }
    SmartDashboard.putBoolean("Found April Tag?", foundObject);

    double distance = 33.6 + (-1.57 * area) + (0.0286 * Math.pow(area, 2));
    SmartDashboard.putNumber("Distance", distance);

    double POVAngle = Math.atan(x / distance)*180 / Math.PI;
    double orbitalAngle = Math.atan(Math.hypot(x, y) / distance)*180 / Math.PI;
    SmartDashboard.putNumber("Orbital Angle", orbitalAngle);
    SmartDashboard.putNumber("POV Angle", POVAngle);

    double velocity = -angController.calculate(Math.copySign(POVAngle, x));
    SmartDashboard.putNumber("Velocity", velocity);
    ChassisSpeeds swerveVelocity = new ChassisSpeeds(0, velocity, 0);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}