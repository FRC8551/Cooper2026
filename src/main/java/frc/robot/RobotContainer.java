// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.ToIntFunction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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

  /**
   * PS5 (DualSense) and Xbox/XInput-style pads report buttons and axes at
   * different raw indices. This holds one device's worth of raw indices for
   * every logical action this robot uses, so the rest of the class doesn't
   * need to know or care which type is actually connected.
   */
  private static final class ControllerMap {
    final int leftXAxis, leftYAxis, rightXAxis, rightYAxis;
    final int leftTriggerAxis, rightTriggerAxis;
    final int zeroGyroButton, comboButtonA, comboButtonB;
    final int reverseIntakeButton, reverseIndexerButton;
    final int pivotZeroButton, pivotSixteenButton;
    final int intakeBackupButton, intakeFullBackupButton;
    final int shooterBackupButton, hubAimToggleButton;

    ControllerMap(int leftXAxis, int leftYAxis, int rightXAxis, int rightYAxis,
        int leftTriggerAxis, int rightTriggerAxis,
        int zeroGyroButton, int comboButtonA, int comboButtonB,
        int reverseIntakeButton, int reverseIndexerButton,
        int pivotZeroButton, int pivotSixteenButton,
        int intakeBackupButton, int intakeFullBackupButton,
        int shooterBackupButton, int hubAimToggleButton) {
      this.leftXAxis = leftXAxis;
      this.leftYAxis = leftYAxis;
      this.rightXAxis = rightXAxis;
      this.rightYAxis = rightYAxis;
      this.leftTriggerAxis = leftTriggerAxis;
      this.rightTriggerAxis = rightTriggerAxis;
      this.zeroGyroButton = zeroGyroButton;
      this.comboButtonA = comboButtonA;
      this.comboButtonB = comboButtonB;
      this.reverseIntakeButton = reverseIntakeButton;
      this.reverseIndexerButton = reverseIndexerButton;
      this.pivotZeroButton = pivotZeroButton;
      this.pivotSixteenButton = pivotSixteenButton;
      this.intakeBackupButton = intakeBackupButton;
      this.intakeFullBackupButton = intakeFullBackupButton;
      this.shooterBackupButton = shooterBackupButton;
      this.hubAimToggleButton = hubAimToggleButton;
    }
  }

  // Indices from edu.wpi.first.wpilibj.XboxController.Button/Axis
  private static final ControllerMap XBOX_MAP = new ControllerMap(
      /* leftX */ 0, /* leftY */ 1, /* rightX */ 4, /* rightY */ 5,
      /* leftTrigger */ 2, /* rightTrigger */ 3,
      /* Y */ 4, /* Back */ 7, /* Start */ 8,
      /* X */ 3, /* A */ 1,
      /* B */ 2, /* Y */ 4,
      /* Back */ 7, /* Start */ 8,
      /* LeftBumper */ 5, /* RightBumper */ 6);

  // Indices from edu.wpi.first.wpilibj.PS5Controller.Button/Axis
  private static final ControllerMap PS5_MAP = new ControllerMap(
      /* leftX */ 0, /* leftY */ 1, /* rightX */ 2, /* rightY */ 5,
      /* L2 */ 3, /* R2 */ 4,
      /* Triangle */ 4, /* Create */ 9, /* Options */ 10,
      /* Square */ 1, /* Cross */ 2,
      /* Circle */ 3, /* Triangle */ 4,
      /* Create */ 9, /* Options */ 10,
      /* L1 */ 5, /* R1 */ 6);

  // Controllers — CommandGenericHID works with any raw HID gamepad.
  private final CommandGenericHID m_driverController = new CommandGenericHID(OIConstants.kDriverControllerPort);
  private final CommandGenericHID m_operatorController = new CommandGenericHID(OIConstants.kOperatorControllerPort);

  /**
   * Re-checked every time it's called, NOT cached. The Driver Station may
   * not have identified the plugged-in device yet at the exact moment
   * RobotContainer is constructed (very early in boot); caching that one
   * read permanently locks in a wrong guess for the whole session. This is
   * cheap (a HAL-cached lookup, no IO) so re-reading it on every button/axis
   * poll (~50 times a second) costs nothing and self-corrects once the DS
   * catches up.
   */
  private static ControllerMap currentMap(int port) {
    return DriverStation.getJoystickIsXbox(port) ? XBOX_MAP : PS5_MAP;
  }

  private Trigger driverButton(ToIntFunction<ControllerMap> selector) {
    return new Trigger(
        () -> m_driverController.getHID().getRawButton(selector.applyAsInt(currentMap(OIConstants.kDriverControllerPort))));
  }

  private Trigger driverAxisGreaterThan(ToIntFunction<ControllerMap> selector, double threshold) {
    return new Trigger(
        () -> m_driverController.getRawAxis(selector.applyAsInt(currentMap(OIConstants.kDriverControllerPort))) > threshold);
  }

  private Trigger operatorButton(ToIntFunction<ControllerMap> selector) {
    return new Trigger(
        () -> m_operatorController.getHID()
            .getRawButton(selector.applyAsInt(currentMap(OIConstants.kOperatorControllerPort))));
  }

  private Trigger operatorAxisGreaterThan(ToIntFunction<ControllerMap> selector, double threshold) {
    return new Trigger(
        () -> m_operatorController.getRawAxis(selector.applyAsInt(currentMap(OIConstants.kOperatorControllerPort))) > threshold);
  }

  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Swerve Input Streams
  // NOTE: these axes are read continuously every loop, so translation and
  // rotation already return to zero (and the robot stops moving/turning)
  // the instant a stick is released — no extra binding needed.
  //
  // Rotation is angular-velocity only now (stick deflection = turn rate).
  // The old "direct angle" mode (stick position = absolute target heading,
  // via withControllerHeadingAxis) was removed — it's notoriously twitchy
  // near center since tiny stick movements swing the commanded heading a
  // lot, and it's what "too much rotation" was almost certainly caused by,
  // on top of the axis-index bug above. Angular-velocity is the standard
  // smooth-360 approach recommended across WPILib/Chief Delphi discussions.
  private final SwerveInputStream m_robotRelative = SwerveInputStream.of(
      m_swerveSubsystem.getSwerveDrive(),
      () -> -m_driverController.getRawAxis(currentMap(OIConstants.kDriverControllerPort).leftYAxis),
      () -> -m_driverController.getRawAxis(currentMap(OIConstants.kDriverControllerPort).leftXAxis))
      .withControllerRotationAxis(
          () -> -m_driverController.getRawAxis(currentMap(OIConstants.kDriverControllerPort).rightXAxis))
      .deadband(OIConstants.kDriverControllerDeadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  private final SwerveInputStream m_allianceRelativeAngularVelocity = m_robotRelative.copy()
      .allianceRelativeControl(true);

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
    // Zero gyro (Xbox Y / PS5 Triangle)
    driverButton(m -> m.zeroGyroButton).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

    // Analog intake: reads live trigger pressure every loop instead of a
    // fixed 0.5, so power scales with how far the trigger is squeezed.
    driverAxisGreaterThan(m -> m.leftTriggerAxis, kTriggerActivationThreshold)
        .whileTrue(m_intakeSubsystem.runIntake(
            () -> m_driverController.getRawAxis(currentMap(OIConstants.kDriverControllerPort).leftTriggerAxis)))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Shooter. Also engages chassis hub-aim while held — see the drive()
    // call in changeDriveMode() below, gated by UserConfig.getHubAimEnabled().
    driverAxisGreaterThan(m -> m.rightTriggerAxis, kTriggerActivationThreshold)
        .whileTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Combo: Xbox Back+Start / PS5 Create+Options
    driverButton(m -> m.comboButtonA).and(driverButton(m -> m.comboButtonB))
        .onTrue(m_swerveSubsystem.zeroGyroWithAllianceCommand());

    // POV/D-pad index is the same convention on basically every gamepad, so
    // this doesn't need to come from the map.
    m_operatorController.povUp().whileTrue(m_intakeSubsystem.setIntakePivotSpeed(0.2));
    m_operatorController.povDown().whileTrue(m_intakeSubsystem.setIntakePivotSpeed(-0.2));

    // NEW: clear analog Intake / Outtake for the operator, mirroring the
    // driver's L2/R2. These were unused on the operator controller before.
    operatorAxisGreaterThan(m -> m.leftTriggerAxis, kTriggerActivationThreshold)
        .whileTrue(m_intakeSubsystem.runIntake(
            () -> m_operatorController.getRawAxis(currentMap(OIConstants.kOperatorControllerPort).leftTriggerAxis)))
        .onFalse(m_intakeSubsystem.stopIntake());

    operatorAxisGreaterThan(m -> m.rightTriggerAxis, kTriggerActivationThreshold)
        .whileTrue(m_intakeSubsystem.runIntake(
            () -> -m_operatorController.getRawAxis(currentMap(OIConstants.kOperatorControllerPort).rightTriggerAxis)))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Reverse intake / secondary outtake (Xbox X / PS5 Square) — kept as a
    // fixed-speed backup alongside the new analog outtake above.
    operatorButton(m -> m.reverseIntakeButton)
        .whileTrue(m_intakeSubsystem.runIntake(-0.65))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Reverse indexers (Xbox A / PS5 Cross)
    operatorButton(m -> m.reverseIndexerButton)
        .whileTrue(m_shooterSubsystem.reverseIndexers())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Pivot to position 0 (Xbox B / PS5 Circle)
    operatorButton(m -> m.pivotZeroButton).onTrue(m_intakeSubsystem.setPivotPosition(0));

    // Pivot to position 16 (Xbox Y / PS5 Triangle)
    operatorButton(m -> m.pivotSixteenButton).onTrue(m_intakeSubsystem.setPivotPosition(16));

    // Intake backup (Xbox Back / PS5 Create)
    operatorButton(m -> m.intakeBackupButton)
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Intake full-power backup (Xbox Start / PS5 Options). Release fully
    // stops the intake.
    operatorButton(m -> m.intakeFullBackupButton)
        .onTrue(m_intakeSubsystem.runIntake(1))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Shooter backup (Xbox LeftBumper / PS5 L1)
    operatorButton(m -> m.shooterBackupButton)
        .onTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Quick in-match hub-aim on/off toggle (Xbox RightBumper / PS5 R1).
    operatorButton(m -> m.hubAimToggleButton).onTrue(new InstantCommand(UserConfig::toggleHubAim));

    // Driver tool: rumble the driver's controller as soon as the flywheel
    // is actually at speed and ready to feed.
    new Trigger(m_shooterSubsystem::isReadyToFire)
        .onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0.6)))
        .onFalse(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0)));
  }

  private void configureDefaultCommands() {
    // Analog pivot fine-control: operator's right stick Y drives the pivot
    // whenever no other pivot command (POV, setPivotPosition) is active.
    m_intakeSubsystem.setDefaultCommand(
        m_intakeSubsystem.manualPivotControl(() -> {
          double stick = -m_operatorController.getRawAxis(currentMap(OIConstants.kOperatorControllerPort).rightYAxis);
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

    // FieldOrientedDirectAngle intentionally falls back to angular-velocity
    // rotation now (see the input-stream comment above) instead of a
    // dedicated heading-snap stream. If an old dashboard save still has
    // that option selected, this keeps it from breaking instead of leaving
    // newInputStream null.
    SwerveInputStream newInputStream = m_robotRelative;

    switch (driveMode) {
      case RobotOriented:
        newInputStream = m_robotRelative;
        break;
      case FieldOrientedAngularVelocity:
      case FieldOrientedDirectAngle:
        newInputStream = m_allianceRelativeAngularVelocity;
        break;
      default:
        break;
    }

    // Shooting also engages chassis hub-aim, same as originally.
    // UserConfig.getHubAimEnabled() (your existing driver-station toggle)
    // still gates whether this actually does anything.
    m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.drive(newInputStream,
            () -> m_driverController
                .getRawAxis(currentMap(OIConstants.kDriverControllerPort).rightTriggerAxis) > kTriggerActivationThreshold));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}