package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.LimelightAlignmentCommand;
import frc.robot.Commands.LimelightFollowCommand;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    private final Joystick m_driveController = new Joystick(0);
    private double m_rotatePower;

    public RobotContainer() {
      m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
          m_drivetrainSubsystem,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.kMaxSpeed,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.kMaxSpeed,
          () -> m_rotatePower * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.kMaxAngularSpeed
      ));

      configureButtons();
    }

    public SequentialCommandGroup autonomousCommands() {
      return new SequentialCommandGroup(
          new PositionDriveCommand(m_drivetrainSubsystem, 1, 0, Math.PI / 2, 0.5, Math.PI / 4)
      );
    }

    private void configureButtons() {
      Button m_resetGyro = new Button(() -> m_driveController.getRawButton(6));
      m_resetGyro.whenPressed(() -> setPose(m_drivetrainSubsystem.getPosition().getX(), m_drivetrainSubsystem.getPosition().getY(), 0));

      Button m_brake = new Button(() -> m_driveController.getRawButton(10));
      m_brake.whenPressed(() -> setIdleMode("brake"));
      m_brake.whenReleased(() -> setIdleMode("coast"));

      Button m_rotateLeft = new Button(() -> m_driveController.getRawButton(11));
      m_rotateLeft.whenPressed(() -> setRotatePower("left"));
      m_rotateLeft.whenReleased(() -> setRotatePower("none"));

      Button m_rotateRight = new Button(() -> m_driveController.getRawButton(12));
      m_rotateRight.whenPressed(() -> setRotatePower("right"));
      m_rotateRight.whenReleased(() -> setRotatePower("none"));

      Button m_limelightAlign = new Button(() -> m_driveController.getRawButton(5));
      m_limelightAlign.whenPressed(() -> new LimelightAlignmentCommand(m_drivetrainSubsystem, m_limelightSubsystem));
      m_limelightAlign.whenReleased(() -> m_drivetrainSubsystem.drive(0, 0, 0, false));

      Button m_limelightFollow = new Button(() -> m_driveController.getRawButton(4));
      m_limelightFollow.whenPressed(() -> new LimelightFollowCommand(m_drivetrainSubsystem, m_limelightSubsystem));
      m_limelightFollow.whenReleased(() -> m_drivetrainSubsystem.drive(0, 0, 0, false));

      SmartDashboard.putData("LimelightCommand", new LimelightAlignmentCommand(m_drivetrainSubsystem, m_limelightSubsystem));
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
      }
      else if (state.equals("right")) {
        m_rotatePower = -0.25;
      }
      else {
        m_rotatePower = 0;
      }
    }
}
