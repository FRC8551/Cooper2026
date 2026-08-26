// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
  private final SparkMax m_pivotMotor = new SparkMax(IntakeConstants.kPivotMotorId, MotorType.kBrushless);

  private boolean m_intakeActive = false;
  private double m_intakeSpeed = 0.5;
  private boolean m_pivotCalibrated = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.closedLoop.feedForward.kV(0.00019);
    intakeConfig.closedLoop.pid(0.0001, 0, 0);
    intakeConfig.inverted(true);
    intakeConfig.smartCurrentLimit(80);
    intakeConfig.closedLoopRampRate(0);

    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig.closedLoop.pid(0.04, 0, 0);
    pivotConfig.inverted(true);

    m_pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
public void periodic() {s
// This method will be called once per scheduler run
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Intake RPM", m_intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Intake Output", m_intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Pivot Position", m_pivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber(IntakeConstants.kSlash + "Pivot Output Current", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putBoolean(IntakeConstants.kSlash + "Pivot Calibrated", m_pivotCalibrated);

if (!m_pivotCalibrated) {
if (m_pivotMotor.getOutputCurrent() > IntakeConstants.kPivotStallCurrentThreshold) {
        m_pivotMotor.getEncoder().setPosition(0);
        m_pivotMotor.set(0);
        m_pivotCalibrated = true;
      } else {
        m_pivotMotor.set(-0.1);
      }
    }

if (m_intakeActive) {
      m_intakeMotor.set(m_intakeSpeed);
    } else {
      m_intakeMotor.set(0);
    }
  }

  public Command runIntake(double speed) {
    return runOnce(() -> {
      m_intakeSpeed = speed;
      m_intakeActive = true;
    });
  }

  /**
   * Analog version of runIntake: reads speed from the supplier every loop
   * instead of locking in a fixed value, so power tracks live trigger
   * pressure (or any other analog input) while the command is running.
   */
  public Command runIntake(DoubleSupplier speedSupplier) {
    return run(() -> {
      m_intakeSpeed = speedSupplier.getAsDouble();
      m_intakeActive = true;
    });
  }

  public Command stopIntake() {
    return runOnce(() -> m_intakeActive = false);
  }

  public Command setIntakePivotSpeed(double speed) {
    return run(() -> m_pivotMotor.set(speed));
  }

  /**
   * Continuous analog pivot control, intended for use as this subsystem's
   * default command. Only touches the motor when the supplier reports a
   * non-zero value — when the input is centered/deadbanded to exactly 0.0,
   * this deliberately does nothing so it can't override a closed-loop
   * position hold left behind by setPivotPosition().
   */
  public Command manualPivotControl(DoubleSupplier speedSupplier) {
    return run(() -> {
      double speed = speedSupplier.getAsDouble();
      if (speed != 0.0) {
        m_pivotMotor.set(speed);
      }
    });
  }

  public Command setPivotPosition(double position) {
    return runOnce(() -> {
      if (m_pivotCalibrated) {
        m_pivotMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
      }
    });
  }
}