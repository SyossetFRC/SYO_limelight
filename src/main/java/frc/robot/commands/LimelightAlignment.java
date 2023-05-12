package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

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
        velocity = angController.calculate(Math.copySign(m_limelightSubsystem.POVAngle, m_limelightSubsystem.x));
        velocity = (Math.abs(velocity) < 0.005) ? 0 : velocity;
        m_drivetrainSubsystem.drive(
                0,
                velocity,
                0,
                true
        );
        SmartDashboard.putNumber("Velocity", velocity);
    }

    public boolean isFinished() {
        if (!(Math.abs(m_limelightSubsystem.POVAngle) < 0.1)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false);
    }
}