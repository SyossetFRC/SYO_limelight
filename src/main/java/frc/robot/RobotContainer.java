package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.DefaultElevatorCommand;
import frc.robot.Commands.IdleDriveCommand;
import frc.robot.Commands.TargetTrackingWithLimelight;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.WinchSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final WinchSubsystem m_winchSubsystem = new WinchSubsystem();

  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(0, true, false, 10, 1.75, 15, 320,
      240, false);

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private double m_powerLimit = 1.0;

  /**
   * This class stores all robot related subsystems, commands, and methods that
   * the {@link Robot} class can utilize during different OpModes.
   */
  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> (-MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.05) / 2.0) * m_powerLimit
            * DrivetrainSubsystem.kMaxAngularSpeed));

    m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(
        m_elevatorSubsystem,
        m_winchSubsystem,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.05),
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05)));

    configureButtons();
  }

  // Currently used for testing kinematics
  public SequentialCommandGroup autonomousCommands() {
    return new SequentialCommandGroup(
        new PositionDriveCommand(m_drivetrainSubsystem, 4.0, 1.0, Math.toRadians(90), 3.50, Math.toRadians(120)),
        new PositionDriveCommand(m_drivetrainSubsystem, 3.0, 0.0, Math.toRadians(45), 3.50, Math.toRadians(120)));
  }

  private void configureButtons() {
    // Driver button A
    Button m_resetPose = new Button(() -> m_driveController.getRawButton(1));
    m_resetPose.whenPressed(() -> setPose(0, 0, 0));

    // Driver button X
    Button m_brake = new Button(() -> m_driveController.getRawButton(3));
    m_brake.whenPressed(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    // Driver D-pad up
    Button m_incrementPowerLimit = new Button(
        () -> (m_driveController.getPOV() >= 315 || m_driveController.getPOV() <= 45));
    m_incrementPowerLimit.whenPressed(() -> changePowerLimit(0.2));

    // Driver D-pad down
    Button m_decrementPowerLimit = new Button(
        () -> (m_driveController.getPOV() >= 135 && m_driveController.getPOV() <= 225));
    m_decrementPowerLimit.whenPressed(() -> changePowerLimit(-0.2));

    // Driver button B
    Button m_translationalLimelightTracking = new Button(() -> m_driveController.getRawButton(2));
    m_translationalLimelightTracking.whileActiveContinuous(
        new TargetTrackingWithLimelight(m_drivetrainSubsystem, m_limelightSubsystem, 0.85, 0.05, 0.025, 0.005, true));
    m_translationalLimelightTracking.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    // Driver button Y
    Button m_rotationalLimelightTracking = new Button(() -> m_driveController.getRawButton(4));
    m_rotationalLimelightTracking.whileActiveContinuous(
        new TargetTrackingWithLimelight(m_drivetrainSubsystem, m_limelightSubsystem, 0.85, 0.05, 0.025, 0.005, false));
    m_rotationalLimelightTracking.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

    Button m_resetElevator = new Button(() -> m_operatorController.getRawButton(7) && m_operatorController.getRawButton(8));
    m_resetElevator.whenPressed(() -> reset());

    SmartDashboard.putData("LimelightTranslationalCommand",
        new TargetTrackingWithLimelight(m_drivetrainSubsystem, m_limelightSubsystem, 0.85, 0.05, 0.025, 0.005, true));
    SmartDashboard.putData("LimelightRotationCommand",
        new TargetTrackingWithLimelight(m_drivetrainSubsystem, m_limelightSubsystem, 0.85, 0.05, 0.025, 0.005, false));
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
  }

  public void setIdleMode(String idleMode) {
    m_drivetrainSubsystem.setIdleMode(idleMode);
  }

  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }

  public void reset() {
    m_elevatorSubsystem.resetEncoders();
    m_winchSubsystem.resetEncoders();
  }
}
