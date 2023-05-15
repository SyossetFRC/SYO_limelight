package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable table;
    public double HRIDAYdistance = 0;
    public double LISULdistance = 0;
    public double FINALdistance = 0;
    public double orbitalAngle = 0;
    public double horizontalAngle = 0;
    public double verticalAngle = 0;
    public double foundTag = 0;
    boolean foundObject = false;
    public double area = 0;
    double xCentimeters = 0;
    double yCentimeters = 0;
    double limelightLensHeightInches = 1.75;
    double goalHeightInches = 15;
    double xResolution = 320;
    double yResolution = 240;
    double xPixels = 0;
    double yPixels = 0;
    double areaPixels = 0;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        // read values periodically
        horizontalAngle = tx.getDouble(0.0);
        verticalAngle = ty.getDouble(0.0);
        foundTag = tv.getDouble(0.0);
        area = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("HAngle", horizontalAngle);
        SmartDashboard.putNumber("VAngle", verticalAngle);
        SmartDashboard.putNumber("Area", area);

        // boolean to check if april tag is found
        if (foundTag != 0.0) {
            foundObject = true;
        } 
        SmartDashboard.putBoolean("?TagFound", foundObject);

        // distance power series
        HRIDAYdistance = Units.inchesToMeters(54.4 * Math.pow(area, -0.475)) * 100; 
        SmartDashboard.putNumber("HDist", HRIDAYdistance);
        
        // distance angular calculations
        LISULdistance = Units.inchesToMeters((goalHeightInches - limelightLensHeightInches) / Math.tan(Math.toRadians(verticalAngle + 10))) * 100;

        FINALdistance = (HRIDAYdistance + LISULdistance) / 2;
        SmartDashboard.putNumber("Final Distance", FINALdistance);

        // centimeter calculations
        xCentimeters = Math.tan(Math.toRadians(horizontalAngle)) * FINALdistance; 
        yCentimeters = Math.tan(Math.toRadians(verticalAngle)) * FINALdistance;
        SmartDashboard.putNumber("XDist (cm)", xCentimeters);
        SmartDashboard.putNumber("YDist (cm)", yCentimeters);

        // pixel calculations
        xPixels = horizontalAngle / 59.6 * xResolution;
        yPixels = verticalAngle / 49.7 * yResolution;
        areaPixels = area / 100 * xResolution * yResolution;
        SmartDashboard.putNumber("xPixels", xPixels);
        SmartDashboard.putNumber("yPixels", yPixels);
        SmartDashboard.putNumber("areaPixels", areaPixels);

         // x angle and orbital angle calculations
         orbitalAngle = Math.toDegrees(Math.hypot(horizontalAngle, verticalAngle));
         SmartDashboard.putNumber("Orbital Angle", orbitalAngle);
         SmartDashboard.putNumber("POV Angle", horizontalAngle);
    }
}