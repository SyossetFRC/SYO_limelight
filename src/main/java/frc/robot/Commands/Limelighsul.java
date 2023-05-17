package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class Limelighsul extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    PIDController m_horizontalPIDController;
    PIDController m_verticalPIDController;
    PIDController m_angController;

    private double m_horizontalVelocity;
    private double m_verticalVelocity;
    private double m_angularVelocity;
    private double distanceToPlayer;
    private double deadband;
    private double kP;

    private boolean fieldRelative;

    public Limelighsul(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem,
            double distanceToPlayer, double kP, double deadband, boolean fieldRelative) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        this.kP = kP;
        this.deadband = deadband;
        this.distanceToPlayer = distanceToPlayer;
        this.fieldRelative = fieldRelative;

        addRequirements(m_drivetrainSubsystem);
    }

    public void initialize() {
        m_horizontalPIDController = new PIDController(kP, 0, 0);
        m_verticalPIDController = new PIDController(kP, 0, 0);
        m_angController = new PIDController(kP, 0, 0);
    }

    public void execute() {
        if (fieldRelative) {
            m_horizontalVelocity = m_horizontalPIDController.calculate(m_limelightSubsystem.getHorizontalAngle());
        } else {
            m_angularVelocity = m_angController.calculate(m_limelightSubsystem.getHorizontalAngle());
        }

        m_verticalVelocity = -m_verticalPIDController.calculate((m_limelightSubsystem.getAreaDistance() -
                distanceToPlayer)*100);

        m_verticalVelocity = (Math.abs(m_verticalVelocity) < deadband) ? 0 : m_verticalVelocity;
        m_horizontalVelocity = (Math.abs(m_horizontalVelocity) < deadband) ? 0 : m_horizontalVelocity;

        m_drivetrainSubsystem.drive(
                m_verticalVelocity,
                m_horizontalVelocity,
                m_angularVelocity,
                fieldRelative);

        SmartDashboard.putNumber("Horizontal Velocity", m_horizontalVelocity);
        SmartDashboard.putNumber("Vertical Velocity", m_verticalVelocity);
        SmartDashboard.putNumber("Angular Velocity", m_angularVelocity);
    }

    public boolean isFinished() {
        if (!(Math.abs(m_limelightSubsystem.getHorizontalAngle()) < 0.25)) 
        {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false);
    }
}