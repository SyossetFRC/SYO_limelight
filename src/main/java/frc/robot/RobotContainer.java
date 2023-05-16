package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.DefaultElevatorCommand;
import frc.robot.Commands.IdleDriveCommand;
import frc.robot.Commands.Limelighsul;
import frc.robot.Commands.Limelighsulevator;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.WinchSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final WinchSubsystem m_winchSubsystem = new WinchSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(0, true, false, 10, 1.75, 15, 320,
      240);

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private double m_rotatePower;

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * (0.75)
            * DrivetrainSubsystem.kMaxSpeed,
        // () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) *
        // (-m_driveController.getRawAxis(3) + 1)
        // * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(2), 0.05) * (0.75)
            * DrivetrainSubsystem.kMaxSpeed,
        // () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(2), 0.05) *
        // (-m_driveController.getRawAxis(3) + 1)
        // * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.05) * (0.5)
            * DrivetrainSubsystem.kMaxAngularSpeed));
    // () -> m_rotatePower * (-m_driveController.getRawAxis(3) + 1) *
    // DrivetrainSubsystem.kMaxAngularSpeed));

    m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(
        m_elevatorSubsystem,
        m_winchSubsystem,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.05),
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05)));
    // NOTE TO DARIUS THIS JOYSTICK STUFF IS JUST FOR TESTING PURPOSES, I JUST WANT
    // TO SEE IF IT'S BETTER THAN THE LOGITECH ONE!

    configureButtons();
  }

  public SequentialCommandGroup autonomousCommands() {
    return new SequentialCommandGroup(
        new PositionDriveCommand(m_drivetrainSubsystem, 3.0, 1.0, Math.toRadians(90), 3, Math.toRadians(90)));
  }

  private void configureButtons() {
    Button m_resetPose = new Button(() -> m_driveController.getRawButton(6));
    m_resetPose.whenPressed(() -> setPose(0, 0, 0));

    Button m_brake = new Button(() -> m_driveController.getRawButton(10));
    m_brake.whenPressed(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    /*
     * Button m_rotateLeft = new Button(() -> m_driveController.getRawButton(11));
     * m_rotateLeft.whenPressed(() -> setRotatePower("left"));
     * m_rotateLeft.whenReleased(() -> setRotatePower("none"));
     * 
     * Button m_rotateRight = new Button(() -> m_driveController.getRawButton(12));
     * m_rotateRight.whenPressed(() -> setRotatePower("right"));
     * m_rotateRight.whenReleased(() -> setRotatePower("none"));
     */

    Button m_limelightFieldRelative = new Button(() -> m_driveController.getRawButton(5));
    m_limelightFieldRelative
        .whenPressed(() -> new Limelighsul(m_drivetrainSubsystem, m_limelightSubsystem, 50.8, 0.025, 0.005, true));

    Button m_limelightRobotRelative = new Button(() -> m_driveController.getRawButton(4));
    m_limelightRobotRelative
        .whenPressed(() -> new Limelighsul(m_drivetrainSubsystem, m_limelightSubsystem, 50.8, 0.025, 0.005, false));

    Button m_limelightElevator = new Button(() -> m_driveController.getRawButton(3));
    m_limelightElevator.whenPressed(() -> new Limelighsulevator(m_winchSubsystem, m_limelightSubsystem));
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
  }

  public void setIdleMode(String idleMode) {
    m_drivetrainSubsystem.setIdleMode(idleMode);
  }

  private void setRotatePower(String state) {
    if (state.equals("left")) {
      m_rotatePower = 0.25;
    } else if (state.equals("right")) {
      m_rotatePower = -0.25;
    } else {
      m_rotatePower = 0;
    }
  }
}
