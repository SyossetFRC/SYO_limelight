package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightFollowCommand extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final PIDController m_distanceController = new PIDController(0.01, 0, 0);
    private final PIDController m_angController = new PIDController(0.01, 0, 0);
    private final double playerDistance = 50.8;
    private double forwardVelocity = 0;
    private double angularVelocity = 0;

    public LimelightFollowCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;
        addRequirements(m_drivetrainSubsystem, m_limelightSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        forwardVelocity = -m_distanceController.calculate(m_limelightSubsystem.HRIDAYdistance - playerDistance);
        angularVelocity = -m_angController.calculate(m_limelightSubsystem.horizontalAngle);

        forwardVelocity = (forwardVelocity < 0.05) ? 0 : forwardVelocity;
        angularVelocity = (angularVelocity < 0.05) ? 0 : angularVelocity;
        
        SmartDashboard.putNumber("Foward Velocity", forwardVelocity);
        SmartDashboard.putNumber("Angular Velocity", angularVelocity);

        m_drivetrainSubsystem.drive(forwardVelocity, 0, angularVelocity, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }

}
