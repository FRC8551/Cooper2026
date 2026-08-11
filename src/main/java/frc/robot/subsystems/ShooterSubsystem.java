// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.UserConfig;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_lowerIndexerMotor = new SparkMax(ShooterConstants.kLowerIndexerMotorId,
      MotorType.kBrushless);
  private final SparkMax m_upperIndexerMotor = new SparkMax(ShooterConstants.kUpperIndexerMotorId,
      MotorType.kBrushless);
  private final SparkFlex m_shooterLeftMotor = new SparkFlex(ShooterConstants.kShooterLeftMotorId,
      MotorType.kBrushless);
  private final SparkFlex m_shooterRightMotor = new SparkFlex(ShooterConstants.kShooterRightMotorId,
      MotorType.kBrushless);

  private boolean m_shooterActive = false;
  private boolean m_flywheelReady = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    SparkMaxConfig lowerIndexerConfig = new SparkMaxConfig();
    SparkMaxConfig upperIndexerConfig = new SparkMaxConfig();

    upperIndexerConfig.apply(lowerIndexerConfig);
    upperIndexerConfig.idleMode(IdleMode.kCoast);

    m_lowerIndexerMotor.configure(lowerIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_upperIndexerMotor.configure(upperIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig shooterRightConfig = new SparkFlexConfig();
    SparkFlexConfig shooterLeftConfig = new SparkFlexConfig();

    shooterRightConfig.closedLoop.feedForward.kV(0.00015);
    shooterRightConfig.closedLoop.p(0.001);
    shooterRightConfig.idleMode(IdleMode.kCoast);
    shooterRightConfig.smartCurrentLimit(120);
    shooterRightConfig.closedLoopRampRate(0);
    shooterRightConfig.inverted(false);

    shooterLeftConfig.apply(shooterRightConfig);
    shooterLeftConfig.inverted(true);

    m_shooterLeftMotor.configure(shooterLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterRightMotor.configure(shooterRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(ShooterConstants.kSlash + "Shooter RPM", m_shooterRightMotor.getEncoder().getVelocity());

    if (m_shooterActive) {
      // Run flywheel — manual tuner number overrides the distance-based
      // table when enabled on the dashboard ("Manual Shooter RPM"). Off the
      // competition field, distance-to-hub isn't meaningful, so this is the
      // way to actually test/tune the shooter at a known RPM.
      double targetRPM = UserConfig.getManualShooterRPMEnabled()
          ? UserConfig.getShooterRPM()
          : getRPM(SwerveSubsystem.getDistanceToHub(SwerveSubsystem.m_robotPose));
      SmartDashboard.putNumber(ShooterConstants.kSlash + "Target RPM", targetRPM);
      double currentRPM = m_shooterRightMotor.getEncoder().getVelocity();

      if (currentRPM < targetRPM - ShooterConstants.kShooterRPMTolerance) {
        // Full send until near setpoint
        m_shooterLeftMotor.setVoltage(12.0);
        m_shooterRightMotor.setVoltage(12.0);
      } else {
        // Hand off to closed loop to hold
        m_shooterLeftMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
        m_shooterRightMotor.getClosedLoopController().setSetpoint(targetRPM, ControlType.kVelocity);
      }

      // Always run upper indexer
      m_upperIndexerMotor.set(0.75);

      // Restored: feed still requires the flywheel at speed AND (if hub-aim
      // is enabled on the driver-station toggle) the chassis being aimed.
      // NEW: also requires the alliance hub to actually be active this
      // shift — fuel scored into an inactive hub is worth 0 points, so
      // there's no reason to dump it through. The flywheel stays spun up
      // regardless, so there's zero delay the instant the hub goes active.
      boolean flywheelReady = m_shooterRightMotor.getEncoder().getVelocity() > targetRPM
          - ShooterConstants.kShooterRPMTolerance;

      boolean aimEnabled = UserConfig.getHubAimEnabled();
      boolean aimed = SwerveSubsystem.isAimedAtHub();
      boolean hubActive = SwerveSubsystem.isHubActive();

      boolean readyToFeed = flywheelReady && (!aimEnabled || aimed) && hubActive;
      m_flywheelReady = readyToFeed;

      if (readyToFeed) {
        m_lowerIndexerMotor.set(0.75);
      } else {
        m_lowerIndexerMotor.set(0.0);
      }
    } else {
      m_flywheelReady = false;
      m_shooterLeftMotor.set(0);
      m_shooterRightMotor.set(0);
      m_lowerIndexerMotor.set(0);
      m_upperIndexerMotor.set(0);
    }
  }

  /** Used by RobotContainer to rumble the driver's controller when a shot is actually ready to feed. */
  public boolean isReadyToFire() {
    return m_flywheelReady;
  }

  private double getRPM(double distanceMeters) {
    // Known data points: {distance, RPM}
    double[][] dataPoints = {
        { 1.6, 2300 },
        { 2.5, 2650 },
        { 3.5, 2900 },
        { 4.0, 3050 },
        { 5.0, 3200 },
        { 5.4, 3400 },
        { 10, 5400 }
    };

    // Clamp to bounds
    if (distanceMeters <= dataPoints[0][0])
      return dataPoints[0][1];
    if (distanceMeters >= dataPoints[dataPoints.length - 1][0])
      return dataPoints[dataPoints.length - 1][1];

    // Find surrounding data points and linearly interpolate
    for (int i = 0; i < dataPoints.length - 1; i++) {
      double d0 = dataPoints[i][0], rpm0 = dataPoints[i][1];
      double d1 = dataPoints[i + 1][0], rpm1 = dataPoints[i + 1][1];

      if (distanceMeters >= d0 && distanceMeters <= d1) {
        double t = (distanceMeters - d0) / (d1 - d0);
        return rpm0 + t * (rpm1 - rpm0);
      }
    }

    return -1; // Unreachable
  }

  public Command runShooter() {
    return runOnce(() -> m_shooterActive = true);
  }

  public Command stopShooter() {
    return runOnce(() -> m_shooterActive = false);
  }

  public Command reverseIndexers() {
    return run(() -> {
      m_lowerIndexerMotor.set(-0.5);
      m_upperIndexerMotor.set(-0.5);
    });
  }
}