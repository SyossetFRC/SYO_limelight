package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;


/*A command that tracks april tags and moves towards them */
public class TargetTrackingWithLimelight extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private PIDController m_horizontalPIDController;
    private PIDController m_verticalPIDController;
    private PIDController m_angController;

    private double m_horizontalVelocity;
    private double m_verticalVelocity;
    private double m_angularVelocity;
    private double distanceToPlayer;
    private double deadband;
    private double kP;
    private double isFinishedTolerance;

    private boolean fieldRelative;

    private ShuffleboardTab limelightTab;

    public TargetTrackingWithLimelight(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem,
            double distanceToPlayer, double isFinishedTolerance, double kP, double deadband, boolean fieldRelative) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        this.distanceToPlayer = distanceToPlayer;
        this.isFinishedTolerance = isFinishedTolerance;
        this.kP = kP;
        this.deadband = deadband;
        this.fieldRelative = fieldRelative;

        limelightTab = Shuffleboard.getTab("Limelight Tracking Command");
        ShuffleboardLayout limelightLayout = limelightTab.getLayout("Limelight Values", BuiltInLayouts.kList)
                .withSize(2, 3).withPosition(0, 0);
        
        limelightLayout.addNumber("Vertical Velocity", () -> m_verticalVelocity);
        limelightLayout.addNumber("Angular Velocity", () -> m_angularVelocity);
        limelightLayout.addNumber("IsFinished() Tolerance", () -> this.isFinishedTolerance);
        limelightLayout.addBoolean("Field Relative", () -> this.fieldRelative);

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        m_horizontalPIDController = new PIDController(kP, 0, 0);
        m_verticalPIDController = new PIDController(kP, 0, 0);
        m_angController = new PIDController(kP, 0, 0);
    }

    public void execute() {
        if (fieldRelative) {
            m_horizontalVelocity = m_horizontalPIDController.calculate(m_limelightSubsystem.getHorizontalTargetAngle());
        } else {
            m_angularVelocity = m_angController.calculate(m_limelightSubsystem.getHorizontalTargetAngle());
        }

        m_verticalVelocity = -m_verticalPIDController.calculate((m_limelightSubsystem.getDistance() -
                distanceToPlayer) * 100);

        m_verticalVelocity = (Math.abs(m_verticalVelocity) < deadband) ? 0 : m_verticalVelocity;
        m_horizontalVelocity = (Math.abs(m_horizontalVelocity) < deadband) ? 0 : m_horizontalVelocity;
        m_angularVelocity = (Math.abs(m_angularVelocity) < deadband) ? 0 : m_horizontalVelocity;

        m_drivetrainSubsystem.drive(
                m_verticalVelocity,
                m_horizontalVelocity,
                m_angularVelocity,
                fieldRelative);

        limelightTab.addNumber("Limelight Distance", () -> m_limelightSubsystem.getDistance());
        limelightTab.addNumber("Limelight Horizontal Target Angle", () -> m_limelightSubsystem.getHorizontalTargetAngle());
        limelightTab.addBoolean("Is Finished", () -> isFinished());
    }

    public boolean isFinished() {
        if (Math.abs(m_limelightSubsystem.getHorizontalTargetAngle()) > isFinishedTolerance
                || (Math.abs(m_limelightSubsystem.getDistance() - distanceToPlayer) > isFinishedTolerance / 5)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}