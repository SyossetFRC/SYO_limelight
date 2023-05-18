package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private NetworkTable networkTable;

    private NetworkTableEntry pipelineEntry;
    private NetworkTableEntry camModeEntry;
    private NetworkTableEntry ledModeEntry;
    private NetworkTableEntry horizontalEntry;
    private NetworkTableEntry verticalEntry;
    private NetworkTableEntry targetAreaEntry;
    private NetworkTableEntry foundTagEntry;

    private ShuffleboardTab limelightTab;
    private GenericEntry ledStatusEntry;
    private GenericEntry cameraStatusEntry;
    private GenericEntry pipelineIdEntry;

    private double currentPipelineId;
    private boolean isLedOn;
    private boolean isCameraModeOn;

    private double horizontalTargetAngle;
    private double verticalTargetAngle;
    private double orbitalAngle;

    private double targetArea;
    private double foundTag;
    private boolean foundTagBool;

    private double targetAreaDistance;
    private double trigDistance;
    private double distance;
    private boolean useTrigForDistanceCalc;

    private double limelightLensHeight;
    private double limelightAngle;
    private double goalHeightInches;

    private double horizontalResolution;
    private double verticalResolution;

    public LimelightSubsystem(double currentPipelineId, boolean isLedOn, boolean isCameraModeOn, double limelightAngle,
            double limelightLensHeight, double goalHeightInches, double horizontalResolution,
            double verticalResolution, boolean useTrigForDistanceCalc) {
        this.currentPipelineId = currentPipelineId;
        this.isLedOn = isLedOn;
        this.isCameraModeOn = isCameraModeOn;

        this.limelightAngle = limelightAngle;
        this.limelightLensHeight = limelightLensHeight;
        this.goalHeightInches = goalHeightInches;

        this.horizontalResolution = horizontalResolution;
        this.verticalResolution = verticalResolution;

        this.useTrigForDistanceCalc = useTrigForDistanceCalc;

        networkTable = NetworkTableInstance.getDefault().getTable("limelight");

        limelightTab = Shuffleboard.getTab("Limelight");
        ShuffleboardLayout limelightLayout = limelightTab
                .getLayout("Limelight Subsystem Specifications", BuiltInLayouts.kList).withSize(2, 3)
                .withPosition(3, 0);

        ledStatusEntry = limelightTab.add("Set LED Status", this.isLedOn)
                .withWidget(BuiltInWidgets.kToggleButton).getEntry();

        cameraStatusEntry = limelightTab.add("Set Camera Status", this.isCameraModeOn)
                .withWidget(BuiltInWidgets.kToggleButton).getEntry();

        pipelineIdEntry = limelightTab.add("Set Pipeline", this.currentPipelineId)
                .withWidget(BuiltInWidgets.kNumberSlider).getEntry();

        limelightTab.addNumber("Horizontal Resolution", () -> this.horizontalResolution);
        limelightTab.addNumber("Vertical Resolution", () -> this.verticalResolution);
        limelightTab.addNumber("Limelight Lens Height", () -> this.limelightLensHeight);
        limelightTab.addNumber("Limelight Angle", () -> this.limelightAngle);
        limelightTab.addNumber("Goal Height", () -> this.goalHeightInches);

        limelightLayout.addNumber("Horizontal Target Angle", () -> getHorizontalTargetAngle());
        limelightLayout.addNumber("Vertical Target Angle", () -> getVerticalTargetAngle());
        limelightLayout.addNumber("Target Area Distance", () -> getTargetAreaDistance());
        limelightLayout.addNumber("Trigonometric Distance", () -> getTrigDistance());
        limelightLayout.addNumber("Preferred Distance", () -> getDistance());
        limelightLayout.addBoolean("Tag Found", () -> getFoundTag());
        limelightLayout.addBoolean("LED Status", () -> getLEDStatus());
        limelightLayout.addBoolean("Camera Status", () -> getCameraModeStatus());
    }

    @Override
    public void periodic() {
        pipelineEntry = networkTable.getEntry("pipeline");
        camModeEntry = networkTable.getEntry("camMode");
        ledModeEntry = networkTable.getEntry("ledMode");
        horizontalEntry = networkTable.getEntry("tx");
        targetAreaEntry = networkTable.getEntry("ta");
        verticalEntry = networkTable.getEntry("ty");
        foundTagEntry = networkTable.getEntry("tv");

        pipelineEntry.setNumber(currentPipelineId);
        if (isCameraModeOn) {
            camModeEntry.setNumber(1);
        } else {
            camModeEntry.setNumber(0);
        }
        if (isLedOn) {
            ledModeEntry.setNumber(3);
        } else {
            ledModeEntry.setNumber(1);
        }
        horizontalTargetAngle = horizontalEntry.getDouble(0.0);
        verticalTargetAngle = verticalEntry.getDouble(0.0);
        targetArea = targetAreaEntry.getDouble(0.0);
        foundTag = foundTagEntry.getDouble(0.0);

        orbitalAngle = Math.toDegrees(Math.hypot(horizontalTargetAngle, verticalTargetAngle));

        foundTagBool = (foundTag != 0) ? true : false;

        targetAreaDistance = Units.inchesToMeters(54.4 * Math.pow(targetArea, -0.475));
        trigDistance = Units.inchesToMeters(
                (goalHeightInches - limelightLensHeight)
                        / Math.tan(Math.toRadians(verticalTargetAngle + limelightAngle)));
        distance = (useTrigForDistanceCalc) ? trigDistance : targetAreaDistance;

        double horizontalDistance = Math.tan(Math.toRadians(horizontalTargetAngle)) * distance;
        double verticalDistance = Math.tan(Math.toRadians(verticalTargetAngle)) * distance;

        double cameraPixelX = horizontalTargetAngle / 59.6 * horizontalResolution;
        double cameraPixelY = verticalTargetAngle / 49.7 * verticalResolution;
        double cameraPixeltargetArea = targetArea / 100 * horizontalResolution * verticalResolution;
    }

    public double getHorizontalTargetAngle() {
        return horizontalTargetAngle;
    }

    public double getVerticalTargetAngle() {
        return verticalTargetAngle;
    }

    public double getTargetAreaDistance() {
        return targetAreaDistance;
    }

    public double getTrigDistance() {
        return trigDistance;
    }

    public double getDistance() {
        return distance;
    }

    public boolean getFoundTag() {
        return foundTagBool;
    }

    public boolean getLEDStatus() {
        return isLedOn;
    }

    public boolean getCameraModeStatus() {
        return isCameraModeOn;
    }

    public double getCurrentPipelineId() {
        return currentPipelineId;
    }

    public void setLEDStatus(boolean isLedOn) {
        this.isLedOn = isLedOn;
    }

    public void setCameraStatus(boolean isCameraModeOn) {
        this.isCameraModeOn = isCameraModeOn;
    }

    public void setPipelineId(double currentPipelineId) {
        this.currentPipelineId = currentPipelineId;
    }
}
