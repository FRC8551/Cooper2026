// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot.DriveMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(m_driverController);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Swerve Input Streams
  private final SwerveInputStream m_robotRelative = SwerveInputStream.of(
      m_swerveSubsystem.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
      .deadband(OIConstants.kDriverControllerDeadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  private final SwerveInputStream m_allianceRelativeAngularVelocity = m_robotRelative.copy()
      .allianceRelativeControl(true);

  private final SwerveInputStream m_allianceRelativeDirectAngle = m_allianceRelativeAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -m_driverController.getRightX(), () -> -m_driverController.getRightY())
      .headingWhile(true);

  // Commands
  private final Command m_driveRobotOrientedAngularVelocity = m_swerveSubsystem
      .drive(m_robotRelative, () -> m_driverController.axisGreaterThan(3, 0.5).getAsBoolean());

  private final Command m_driveFieldOrientedAngularVelocity = m_swerveSubsystem
      .drive(m_allianceRelativeAngularVelocity, () -> m_driverController.axisGreaterThan(3, 0.5).getAsBoolean());

  private final Command m_driveFieldOrientedDirectAngle = m_swerveSubsystem
      .drive(m_allianceRelativeDirectAngle, () -> m_driverController.axisGreaterThan(3, 0.5).getAsBoolean());

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // Interstellar reference
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Before you get all teary...",
        "Try to remember that as a robot, I have to do anything you say. Good luck, Cooper."));

    registerNamedCommands();
    configureBindings();

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Nothing", Commands.none());
  }

  private void configureBindings() {
    m_driverController.y().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

    m_driverController.axisGreaterThan(2, 0.5)
        .onTrue(new InstantCommand(() -> m_intakeSubsystem.runIntake()))
        .onFalse(new InstantCommand(() -> m_intakeSubsystem.stopIntake()));

    m_driverController.axisGreaterThan(3, 0.5)
        .whileTrue(new InstantCommand(() -> m_shooterSubsystem.runShooter()).repeatedly())
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.stopShooter()));

    m_operatorController.povUp().onTrue(new InstantCommand(() -> m_intakeSubsystem.setIntakePivotSpeed(0.2)))
        .onFalse(new InstantCommand(() -> m_intakeSubsystem.setIntakePivotSpeed(0)));

    m_operatorController.povDown().onTrue(new InstantCommand(() -> m_intakeSubsystem.setIntakePivotSpeed(-0.2)))
        .onFalse(new InstantCommand(() -> m_intakeSubsystem.setIntakePivotSpeed(0)));
  }

  public void changeDriveMode(DriveMode driveMode) {
    if (m_swerveSubsystem.getCurrentCommand() != null) {
      m_swerveSubsystem.getCurrentCommand().cancel();
    }

    switch (driveMode) {
      case RobotOriented:
        m_swerveSubsystem.setDefaultCommand(m_driveRobotOrientedAngularVelocity);
        break;
      case FieldOrientedAngularVelocity:
        m_swerveSubsystem.setDefaultCommand(m_driveFieldOrientedAngularVelocity);
        break;
      case FieldOrientedDirectAngle:
        m_swerveSubsystem.setDefaultCommand(m_driveFieldOrientedDirectAngle);
      default:
        break;
    }
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
