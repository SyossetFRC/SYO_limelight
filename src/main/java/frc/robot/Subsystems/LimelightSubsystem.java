package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.GenericEntry;

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

    private GenericEntry m_txEntry;
    private GenericEntry m_tyEntry;
    private GenericEntry m_taEntry;

    private GenericEntry m_RegressionDistanceEntry;
    private GenericEntry m_TrigDistanceEntry;
    private GenericEntry m_AvgDistaanceEntry;

    private GenericEntry m_OrbitalAngleEntry;
    private GenericEntry m_FoundTagEntry;

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
        instantiateShuffleboard();
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
        if (cameraMode) {
            camModeEntry.setNumber(1);
        } else {
            camModeEntry.setNumber(0);
        }
        if (ledOn) {
            ledModeEntry.setNumber(3);
        } else {
            ledModeEntry.setNumber(1);
        }
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
        SmartDashboard.putNumber("Area Distance", areaDistance);
        SmartDashboard.putNumber("Horizontal Angle", horizontalAngle);
        SmartDashboard.putNumber("Vertical Angle", verticalAngle);

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

    public void setLEDStatus(boolean ledOn) {
        this.ledOn = ledOn;
    }

    public boolean getLEDStatus() {
        return ledOn;
    }

    public void setPipeline(double pipelineId) {
        this.pipelineId = pipelineId;
    }

    public void instantiateShuffleboard() {
        
    }

    public void updateShuffleboard() {
       
    }
}