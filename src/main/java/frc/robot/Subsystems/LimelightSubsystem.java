package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.GenericEntry;


 import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
 import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
 import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
 import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable networkTable;

    private double pipelineId;
    private boolean ledOn;
    private boolean cameraMode;

    private double horizontalAngle;
    private double verticalAngle;
    private double orbitalAngle;

    private double area;
    private double foundTag;
    private boolean foundTagBool;

    private double areaDistance;
    private double trigDistance;
    private double avgDistance;

    private double limelightLensHeight = 1.75;
    private double limelightAngle;
    private double goalHeightInches = 15;

    private double horizontalResolution;
    private double verticalResolution;

    private final GenericEntry m_txEntry;
    private final GenericEntry m_tyEntry;
    private final GenericEntry m_taEntry;

    private final GenericEntry m_RegressionDistanceEntry;
    private final GenericEntry m_TrigDistanceEntry;
    private final GenericEntry m_AvgDistaanceEntry;

    private final GenericEntry m_OrbitalAngleEntry;
    private final GenericEntry m_FoundTagEntry;


    public LimelightSubsystem(double pipelineId, boolean ledOn, boolean cameraMode, double limelightAngle,
            double limelightLensHeight, double goalHeightInches, double horizontalResolution,
            double verticalResolution) {
        this.pipelineId = pipelineId;
        this.ledOn = ledOn;
        this.cameraMode = cameraMode;

        this.limelightAngle = limelightAngle;
        this.limelightLensHeight = limelightLensHeight;
        this.goalHeightInches = goalHeightInches;

        this.horizontalResolution = horizontalResolution;
        this.verticalResolution = verticalResolution;

        networkTable = NetworkTableInstance.getDefault().getTable("limelight");

        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

         ShuffleboardLayout NetworkTables = tab.getLayout("NetworkTables", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 0);
         m_txEntry = NetworkTables.add("tx(POV)", networkTable.getEntry("tx")).getEntry();
         m_tyEntry = NetworkTables.add("ty",networkTable.getEntry("ty")).getEntry();
         m_taEntry = NetworkTables.add("ta",networkTable.getEntry("ta")).getEntry();

         ShuffleboardLayout Distances = tab.getLayout("Angles", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 0);
         m_RegressionDistanceEntry = Distances.add("Regression Distance", 0).getEntry();
         m_TrigDistanceEntry = Distances.add("Trig Distance", 0).getEntry();
         m_AvgDistaanceEntry = Distances.add("Average Distance", 0).getEntry();
         

         ShuffleboardLayout Other = tab.getLayout("Other", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
         m_OrbitalAngleEntry = Other.add("Orbital Angle", 0).getEntry();
         m_FoundTagEntry = Other.add("Found Tag?", false).getEntry();
    }

    public void updateShuffleboard()
    {
        m_txEntry.setDouble(horizontalAngle);
        m_tyEntry.setDouble(verticalAngle);
        m_taEntry.setDouble(area);

        m_RegressionDistanceEntry.setDouble(areaDistance);
        m_TrigDistanceEntry.setDouble(trigDistance);
        m_AvgDistaanceEntry.setDouble(avgDistance);
     

        m_OrbitalAngleEntry.setDouble(orbitalAngle);
        m_FoundTagEntry.setBoolean(foundTagBool);

    }


    @Override
    public void periodic() {
        NetworkTableEntry pipelineEntry = networkTable.getEntry("pipeline");
        NetworkTableEntry camModeEntry = networkTable.getEntry("camMode");
        NetworkTableEntry ledModeEntry = networkTable.getEntry("ledMode");
        NetworkTableEntry horizontalEntry = networkTable.getEntry("tx");
        NetworkTableEntry verticalEntry = networkTable.getEntry("ty");
        NetworkTableEntry areaEntry = networkTable.getEntry("ta");
        NetworkTableEntry foundTagEntry = networkTable.getEntry("tv");

        pipelineEntry.setNumber(pipelineId);
        if (cameraMode) {camModeEntry.setNumber(1);} else {camModeEntry.setNumber(0);}
        if (ledOn) {ledModeEntry.setNumber(3);} else {ledModeEntry.setNumber(1);}
        horizontalAngle = horizontalEntry.getDouble(0.0);
        verticalAngle = verticalEntry.getDouble(0.0);
        area = areaEntry.getDouble(0.0);
        foundTag = foundTagEntry.getDouble(0.0);

        orbitalAngle = Math.toDegrees(Math.hypot(horizontalAngle, verticalAngle));

        foundTagBool = (foundTag != 0) ? true : false;

        areaDistance = Units.inchesToMeters(54.4 * Math.pow(area, -0.475));
        /* TODO: Calculate trigonometric distances */
        trigDistance = Units.inchesToMeters(
                (goalHeightInches - limelightLensHeight) / Math.tan(Math.toRadians(verticalAngle + limelightAngle)));
        avgDistance = (areaDistance + trigDistance) / 2;

        double horizontalDistance = Math.tan(Math.toRadians(horizontalAngle)) * areaDistance;
        double verticalDistance = Math.tan(Math.toRadians(verticalAngle)) * areaDistance;


        double cameraPixelX = horizontalAngle / 59.6 * horizontalResolution;
        double cameraPixelY = verticalAngle / 49.7 * verticalResolution;
        double cameraPixelArea = area / 100 * horizontalResolution * verticalResolution;

        updateShuffleboard();
    }
    public double getHorizontalAngle() {
        return horizontalAngle;
    }

    public double getVerticalAngle() {
        return verticalAngle;
    }

    public double getAreaDistance() {
        return areaDistance;
    }

    public double getTrigDistance() {
        return areaDistance;
    }

    public boolean getFoundTag() {
        return foundTagBool;
    }

}