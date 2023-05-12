package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable table;
    boolean foundObject = false;
    public double distance = 0;
    public double POVAngle = 0;
    public double orbitalAngle = 0;
    public double x = 0;
    public double y = 0;
    public double area = 0;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        // read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // boolean to check if april tag is found
        if (x != 0) {
            foundObject = true;
        } else {
            foundObject = false;
        }
        SmartDashboard.putBoolean("Found April Tag?", foundObject);

        // distance polynomial reg
        distance = 33.6 + (-1.57 * area) + (0.0286 * Math.pow(area, 2));
        SmartDashboard.putNumber("Distance", distance);

        // x angle and orbital angle calculations
        POVAngle = Math.atan(x / distance) * 180 / Math.PI;
        orbitalAngle = Math.atan(Math.hypot(x, y) / distance) * 180 / Math.PI;
        SmartDashboard.putNumber("Orbital Angle", orbitalAngle);
        SmartDashboard.putNumber("POV Angle", POVAngle);
    }

    public double getPOVAngle() {
        return POVAngle;
    }
}