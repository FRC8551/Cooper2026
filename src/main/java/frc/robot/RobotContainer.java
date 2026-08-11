// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.UserConfig.DriveMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // How far a trigger has to move before we consider it "pressed." Kept low
  // so analog intake power feels responsive from a light squeeze.
  private static final double kTriggerActivationThreshold = 0.05;

  // Deadband + max speed for the operator's analog pivot fine-control stick.
  private static final double kPivotStickDeadband = 0.1;
  private static final double kPivotStickMaxSpeed = 0.3;

  // Controllers
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
      OIConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Swerve Input Streams
  // NOTE: getLeftX/getLeftY/getRightX are read continuously every loop, so
  // translation and rotation already return to zero (and the robot stops
  // moving/turning) the instant a stick is released — no extra binding needed.
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
      .withControllerHeadingAxis(
          () -> m_driverController.getRightX() * (m_swerveSubsystem.isRedAlliance() ? 1 : -1),
          () -> m_driverController.getRightY() * (m_swerveSubsystem.isRedAlliance() ? 1 : -1))
      .headingWhile(true);

  // Commands

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // Interstellar reference
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Before you get all teary...",
        "Try to remember that as a robot, I have to do anything you say. Good luck, Cooper."));

    registerNamedCommands();
    configureBindings();
    configureDefaultCommands();

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Run Intake", m_intakeSubsystem.runIntake(0.65));
    NamedCommands.registerCommand("Stop Intake", m_intakeSubsystem.stopIntake());
    NamedCommands.registerCommand("Run Shooter", m_shooterSubsystem.runShooter());
    NamedCommands.registerCommand("Stop Shooter", m_shooterSubsystem.stopShooter());
  }

  private void configureBindings() {
    // Xbox Y -> PS5 Triangle
    m_driverController.triangle().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

    // Analog intake: reads live trigger pressure every loop instead of a
    // fixed 0.5, so power scales with how far L2 is squeezed.
    m_driverController.axisGreaterThan(PS5Controller.Axis.kL2.value, kTriggerActivationThreshold)
        .whileTrue(m_intakeSubsystem.runIntake(m_driverController::getL2Axis))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Shooter. Also engages chassis hub-aim while held — see the drive()
    // call in changeDriveMode() below, gated by UserConfig.getHubAimEnabled().
    m_driverController.axisGreaterThan(PS5Controller.Axis.kR2.value, kTriggerActivationThreshold)
        .whileTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Xbox Back+Start -> PS5 Create+Options
    m_driverController.create().and(m_driverController.options()).onTrue(m_swerveSubsystem.zeroGyroWithAllianceCommand());

    // POV up/down converted to whileTrue-only (no onFalse). Releasing the
    // button now simply interrupts this command and hands control straight
    // back to the pivot's default command (the analog stick control below)
    // instead of leaving a permanent "hold at 0%" command occupying the
    // subsystem forever.
    m_operatorController.povUp().whileTrue(m_intakeSubsystem.setIntakePivotSpeed(0.2));
    m_operatorController.povDown().whileTrue(m_intakeSubsystem.setIntakePivotSpeed(-0.2));

    // Xbox X -> PS5 Square
    m_operatorController.square().whileTrue(m_intakeSubsystem.runIntake(-0.65))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Xbox A(button 1) -> PS5 Cross
    m_operatorController.cross().whileTrue(m_shooterSubsystem.reverseIndexers())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Was previously ALSO bound to A (a()) in the Xbox version -> double bind.
    // Moved to Circle, which was unused, so Cross and Circle each do one thing.
    m_operatorController.circle().onTrue(m_intakeSubsystem.setPivotPosition(0));

    // Xbox Y -> PS5 Triangle
    m_operatorController.triangle().onTrue(m_intakeSubsystem.setPivotPosition(16));

    // Xbox button(7)=Back -> PS5 Create — intake backup
    m_operatorController.create()
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Xbox button(8)=Start -> PS5 Options — intake backup
    m_operatorController.options().onTrue(m_intakeSubsystem.runIntake(1)).onFalse(m_intakeSubsystem.stopIntake());

    // NEW: operator shooter backup, previously missing. L1 was unused.
    m_operatorController.L1()
        .onTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    // NEW: quick in-match hub-aim on/off toggle. R1 was unused. This flips
    // UserConfig's button override — the dashboard "Hub Aim" chooser needs
    // to stay set to Enabled for this to have any effect (see UserConfig).
    m_operatorController.R1().onTrue(new InstantCommand(UserConfig::toggleHubAim));

    // Driver tool: rumble the driver's controller as soon as the flywheel
    // is actually at speed and ready to feed, so they get a physical cue
    // instead of having to watch the dashboard RPM readout.
    new Trigger(m_shooterSubsystem::isReadyToFire)
        .onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0.6)))
        .onFalse(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0)));
  }

  private void configureDefaultCommands() {
    // Analog pivot fine-control: operator's right stick Y drives the pivot
    // whenever no other pivot command (POV, setPivotPosition) is active.
    // Only actuates the motor when the stick is meaningfully off-center —
    // when centered it does nothing at all, rather than calling set(0),
    // so it can't stomp on a closed-loop position hold from
    // setPivotPosition() (circle/triangle) the moment the command frees up.
    m_intakeSubsystem.setDefaultCommand(
        m_intakeSubsystem.manualPivotControl(() -> {
          double stick = -m_operatorController.getRightY();
          if (Math.abs(stick) < kPivotStickDeadband) {
            return 0.0;
          }
          return stick * kPivotStickMaxSpeed;
        }));
  }

  public void changeDriveMode(DriveMode driveMode) {
    if (m_swerveSubsystem.getCurrentCommand() != null) {
      m_swerveSubsystem.getCurrentCommand().cancel();
    }

    SwerveInputStream newInputStream = null;

    switch (driveMode) {
      case RobotOriented:
        newInputStream = m_robotRelative;
        break;
      case FieldOrientedAngularVelocity:
        newInputStream = m_allianceRelativeAngularVelocity;
        break;
      case FieldOrientedDirectAngle:
        newInputStream = m_allianceRelativeDirectAngle;
        break;
      default:
        break;
    }

    // Restored: shooting also engages chassis hub-aim, same as originally.
    // UserConfig.getHubAimEnabled() (your existing driver-station toggle)
    // still gates whether this actually does anything.
    m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.drive(newInputStream,
            () -> m_driverController.axisGreaterThan(PS5Controller.Axis.kR2.value, kTriggerActivationThreshold)
                .getAsBoolean()));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}