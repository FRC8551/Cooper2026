// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final VictorSPX m_lowerIndexerMotor = new VictorSPX(ShooterConstants.kLowerIndexerMotorId);
  private final VictorSPX m_upperIndexerMotor = new VictorSPX(ShooterConstants.kUpperIndexerMotorId);
  private final SparkFlex m_shooterLeftMotor = new SparkFlex(ShooterConstants.kShooterLeftMotorId,
      MotorType.kBrushless);
  private final SparkFlex m_shooterRightMotor = new SparkFlex(ShooterConstants.kShooterRightMotorId,
      MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    FeedForwardConfig ffConfig = new FeedForwardConfig();
    ffConfig.kV(0.00015);

    SparkFlexConfig leftConfig = new SparkFlexConfig();

    leftConfig.inverted(true);
    leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.closedLoop.pid(0.001, 0, 0);
    leftConfig.closedLoop.feedForward.apply(ffConfig);

    SparkFlexConfig rightConfig = new SparkFlexConfig();

    rightConfig.inverted(false);
    rightConfig.idleMode(IdleMode.kCoast);
    rightConfig.closedLoop.pid(0.001, 0, 0);
    rightConfig.closedLoop.feedForward.apply(ffConfig);

    m_shooterLeftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterRightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(ShooterConstants.kSlash + "Shooter RPM", m_shooterRightMotor.getEncoder().getVelocity());
  }

  public void runShooter() {
    m_shooterLeftMotor.getClosedLoopController().setSetpoint(3900, ControlType.kVelocity);
    m_shooterRightMotor.getClosedLoopController().setSetpoint(3900,
        ControlType.kVelocity);
    if (m_shooterRightMotor.getEncoder().getVelocity() >= 3700) {
      m_lowerIndexerMotor.set(VictorSPXControlMode.PercentOutput, 0.7);
      m_upperIndexerMotor.set(VictorSPXControlMode.PercentOutput, 0.85);
    } else {
      m_lowerIndexerMotor.set(VictorSPXControlMode.PercentOutput, 0.31);
      m_upperIndexerMotor.set(VictorSPXControlMode.PercentOutput, 0.85);
    }
  }

  public void stopShooter() {
    m_shooterLeftMotor.set(0);
    m_shooterRightMotor.set(0);
    m_lowerIndexerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    m_upperIndexerMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }
}
