package frc.robot.Commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.WinchSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;


import edu.wpi.first.math.controller.PIDController;

public class WinchElevatorLimelightCommand extends CommandBase{
    LimelightSubsystem m_limelightSubsystem;
    WinchSubsystem m_winchSubsystem;
    ElevatorSubsystem m_elevatorSubsystem;


    private double targetDistance;
    private double horizontalAngle;
    private double verticalAngle;
    private double elevatorDistance;// this is the distance the elevator should be away from april tag
    private double elevatorOffset; //distance limelight to unextended elevator end point
    private double winchAngle; // stores the current angle of the winch
    private double elevatorCurrentExtension; // stores how far elevator extended
    final private double winchPower = 0.5;
    final private double elevatorPower = 0.5;

    private boolean winchEnabled = false;
    private boolean elevatorEnabled = false;
    private boolean winchFinished = true;
    private boolean elevatorFinished = true;

    private PIDController winchPID;
    private PIDController elevatorPID;

    //winch and elevator
    public WinchElevatorLimelightCommand(WinchSubsystem winchSubsystem, LimelightSubsystem limelightSusbsystem, ElevatorSubsystem elevatorSubsystem, double elevDistance)
    {
        m_limelightSubsystem = limelightSusbsystem;
        m_winchSubsystem = winchSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;

        winchEnabled = true;
        elevatorEnabled = true;
        elevatorDistance = elevDistance;

        winchFinished = false;
        elevatorFinished = false;

        winchPID = new PIDController(0.05, 0, 0);
        elevatorPID = new PIDController(0.05, 0, 0);

        addRequirements(winchSubsystem, limelightSusbsystem, elevatorSubsystem);
    }
    //just winch
    public WinchElevatorLimelightCommand(WinchSubsystem winchSubsystem, LimelightSubsystem limelightSubsystem)
    {
        m_winchSubsystem = winchSubsystem;
        winchEnabled = true;
        elevatorEnabled = false;
        winchFinished = false;

        winchPID = new PIDController(0.05, 0, 0);

        addRequirements(winchSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize()
    {
        
        
    }

    @Override
    public void execute()
    {
        targetDistance = m_limelightSubsystem.getTargetAreaDistance();
        horizontalAngle = m_limelightSubsystem.getHorizontalTargetAngle();
        verticalAngle = m_limelightSubsystem.getVerticalTargetAngle();

        if(winchEnabled)
        {
            winchAngle = m_winchSubsystem.getWinchAbsPosition();

            if(winchAngle < 65 && winchAngle > 30 && Math.abs(verticalAngle) > 2)
            {
                m_winchSubsystem.rotate(Math.copySign(winchPower, (verticalAngle - m_winchSubsystem.getWinchAbsPosition())));

                //m_winchSubsystem.rotate(winchPID.calculate(winchAngle, 0)); PID
            }
            else
            {
                m_winchSubsystem.rotate(0);
            }
        }
        if(elevatorEnabled)
        {
            elevatorCurrentExtension = m_elevatorSubsystem.getElevatorAbsPosition();

            if(elevatorCurrentExtension >0.15 && elevatorCurrentExtension < 0.85 && Math.abs(targetDistance- elevatorOffset - elevatorDistance)>2)
            {
                m_elevatorSubsystem.extend(Math.copySign(elevatorPower, (targetDistance- elevatorOffset - elevatorDistance) - elevatorCurrentExtension));
            }
            else
            {
                m_elevatorSubsystem.extend(0);
            }
        }
    }

    @Override
    public boolean isFinished()
    {
        return elevatorFinished && winchFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_winchSubsystem.rotate(0);
        m_elevatorSubsystem.extend(0);
    }
    
}
