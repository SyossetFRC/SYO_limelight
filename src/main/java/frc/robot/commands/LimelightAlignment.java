package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightAlignment extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    PIDController angController;
    private double velocity;

    public LimelightAlignment(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;
        addRequirements(m_limelightSubsystem, m_drivetrainSubsystem);
    }

    public void initialize() {
        angController = new PIDController(0.01, 0, 0);
        velocity = 0;
    }

    public void execute() {
        velocity = -angController.calculate(Math.copySign(m_limelightSubsystem.POVAngle, m_limelightSubsystem.x));
        velocity = (velocity < 0.005) ? 0 : velocity;
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        velocity,
                        0,
                        m_drivetrainSubsystem.getGyroscopeRotation()));
        SmartDashboard.putNumber("Velocity", velocity);
    }

    public boolean isFinished() {
        if (!(Math.abs(m_limelightSubsystem.POVAngle) < 0.1) || velocity != 0) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_drivetrainSubsystem.updateDistance();
    }
}
