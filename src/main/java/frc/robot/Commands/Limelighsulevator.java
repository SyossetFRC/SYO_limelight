package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.WinchSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class Limelighsulevator extends CommandBase {
    private final WinchSubsystem m_winchSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;

    private double m_startAngle;

    public Limelighsulevator(WinchSubsystem winchSubsystem, LimelightSubsystem limelightSubsystem){
        m_winchSubsystem = winchSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        addRequirements(winchSubsystem);
    }

    @Override
    public void initialize() {
        m_startAngle = m_winchSubsystem.getWinchAbsPosition();
        m_winchSubsystem.rotate(Math.copySign(0.8, (m_limelightSubsystem.getVerticalAngle() - m_startAngle)));
    }

    @Override
    public boolean isFinished() {
        if (!(Math.abs(m_limelightSubsystem.getVerticalAngle() - m_winchSubsystem.getWinchAbsPosition()) < 2.5)) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_winchSubsystem.rotate(0);
    } 
}
