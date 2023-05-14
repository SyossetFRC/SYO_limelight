package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightAlignment extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    PIDController horizontalPIDController;
    PIDController verticalPIDController;
    private double horizontalVelocity;
    private double verticalVelocity;
    private double playerDistance;

    public LimelightAlignment(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;
        addRequirements(m_limelightSubsystem, m_drivetrainSubsystem);
    }

    public void initialize() {
        horizontalPIDController = new PIDController(0.01, 0, 0);
        verticalPIDController = new PIDController(0.01, 0, 0);
        horizontalVelocity = 0;
        verticalVelocity = 0;
        playerDistance = 50.8;
    }

    public void execute() {
        horizontalVelocity = horizontalPIDController.calculate(m_limelightSubsystem.horizontalAngle);
        //verticalVelocity = -verticalPIDController.calculate(m_limelightSubsystem.HRIDAYdistance - playerDistance);
        horizontalVelocity ```= (Math.abs(horizontalVelocity) < 0.005) ? 0 : horizontalVelocity;
        m_drivetrainSubsystem.drive(
                verticalVelocity,
                horizontalVelocity,
                0,
                true
        );
        SmartDashboard.putNumber("Horizontal Velocity", horizontalVelocity);
        SmartDashboard.putNumber("Vertical Velocity", verticalVelocity);
    }

    public boolean isFinished() {
        if (!(Math.abs(m_limelightSubsystem.horizontalAngle) < 0.1)) 
        //&& Math.abs(m_limelightSubsystem.HRIDAYdistance - playerDistance) < 1) 
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