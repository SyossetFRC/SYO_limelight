package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

    private final GenericEntry m_txEntry;
    private final GenericEntry m_tyEntry;
    private final GenericEntry m_taEntry;

    private final GenericEntry m_RegressionDistanceEntry;
    private final GenericEntry m_TrigDistanceEntry;
    private final GenericEntry m_AvgDistaanceEntry;
    private final GenericEntry m_XDistanceCmEntry;
    private final GenericEntry m_YDistanceCmEntry;

    private final GenericEntry m_xPixelsEntry;
    private final GenericEntry m_yPixelsEntry;
    private final GenericEntry m_areaPixelsEntry;

    private final GenericEntry m_OrbitalAngleEntry;
    private final GenericEntry m_FoundTagEntry;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

        ShuffleboardLayout NetworkTables = tab.getLayout("NetworkTables", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 0);
        m_txEntry = NetworkTables.add("tx(POV)", table.getEntry("tx")).getEntry();
        m_tyEntry = NetworkTables.add("ty",table.getEntry("ty")).getEntry();
        m_taEntry = NetworkTables.add("ta",table.getEntry("ta")).getEntry();

        ShuffleboardLayout Distances = tab.getLayout("Angles", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 0);
        m_RegressionDistanceEntry = Distances.add("Regression Distance", 0).getEntry();
        m_TrigDistanceEntry = Distances.add("Trig Distance", 0).getEntry();
        m_AvgDistaanceEntry = Distances.add("Average Distance", 0).getEntry();
        m_XDistanceCmEntry = Distances.add("X Distance (cm)", 0).getEntry();
        m_YDistanceCmEntry = Distances.add("Y Distance (cm)", 0).getEntry();
        
        ShuffleboardLayout Pixels = tab.getLayout("Pixels", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
        m_xPixelsEntry = Pixels.add("X Pixels", 0).getEntry();
        m_yPixelsEntry = Pixels.add("Y Pixels", 0).getEntry();
        m_areaPixelsEntry = Pixels.add("Area Pixels", 0).getEntry();

        ShuffleboardLayout Other = tab.getLayout("Other", BuiltInLayouts.kList).withSize(2, 2).withPosition(6, 0);
        m_OrbitalAngleEntry = Other.add("Orbital Angle", 0).getEntry();
        m_FoundTagEntry = Other.add("Found Tag?", false).getEntry();
    }

    public void updateShuffleboard()
    {
        m_txEntry.setDouble(horizontalAngle);
        m_tyEntry.setDouble(verticalAngle);
        m_taEntry.setDouble(area);

        m_RegressionDistanceEntry.setDouble(HRIDAYdistance);
        m_TrigDistanceEntry.setDouble(LISULdistance);
        m_AvgDistaanceEntry.setDouble(FINALdistance);
        m_XDistanceCmEntry.setDouble(xCentimeters);
        m_YDistanceCmEntry.setDouble(yCentimeters);

        m_xPixelsEntry.setDouble(xPixels);
        m_yPixelsEntry.setDouble(yPixels);
        m_areaPixelsEntry.setDouble(areaPixels);

        m_OrbitalAngleEntry.setDouble(orbitalAngle);
        m_FoundTagEntry.setBoolean(foundObject);

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

        

        // boolean to check if april tag is found
        if (foundTag != 0.0) {
            foundObject = true;
        } 
        SmartDashboard.putBoolean("?TagFound", foundObject);

        // distance power series
        HRIDAYdistance = Units.inchesToMeters(54.4 * Math.pow(area, -0.475)) * 100; 
        
        // distance angular calculations
        LISULdistance = Units.inchesToMeters((goalHeightInches - limelightLensHeightInches) / Math.tan(Math.toRadians(verticalAngle + 10))) * 100;

        FINALdistance = (HRIDAYdistance + LISULdistance) / 2;

        // centimeter calculations
        xCentimeters = Math.tan(Math.toRadians(horizontalAngle)) * FINALdistance; 
        yCentimeters = Math.tan(Math.toRadians(verticalAngle)) * FINALdistance;
        

        // pixel calculations
        xPixels = horizontalAngle / 59.6 * xResolution;
        yPixels = verticalAngle / 49.7 * yResolution;
        areaPixels = area / 100 * xResolution * yResolution;
    
         // x angle and orbital angle calculations
         orbitalAngle = Math.toDegrees(Math.hypot(horizontalAngle, verticalAngle));

         updateShuffleboard();
    }
}