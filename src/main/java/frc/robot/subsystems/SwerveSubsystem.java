// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UserConfig;
import frc.robot.Constants.SwerveConstants;
import frc.robot.UserConfig.DriveMode;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerve;

  private final PIDController m_aimController = new PIDController(0.08, 0, 0.002);

  private static boolean m_isAimedAtHub = false;
  public static Pose2d m_robotPose = new Pose2d();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerve = new SwerveParser(SwerveConstants.kSwerveConfigurationDirectory)
          .createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Swerve configuration
    m_swerve.setAngularVelocityCompensation(true, true, 0.1);
    m_swerve.setAutoCenteringModules(false);
    m_swerve.setChassisDiscretization(true, 0.02); // Change this to true
    m_swerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    m_swerve.setHeadingCorrection(true);
    m_swerve.setModuleEncoderAutoSynchronize(true, 1);
    m_swerve.setModuleStateOptimization(true);
    m_swerve.setMotorIdleMode(false);

    if (SwerveDriveTelemetry.isSimulation) {
      m_swerve.resetOdometry(new Pose2d(2.5, 4.030, Rotation2d.fromDegrees(0)));
    }

    setupPathPlanner();

    m_aimController.enableContinuousInput(-180, 180);
    // m_aimController.setTolerance(15);
  }

  @Override
  public void periodic() {
    ChassisSpeeds robotVelocity = m_swerve.getRobotVelocity();
    double velocity = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) * 2.23694;
    SmartDashboard.putNumber(SwerveConstants.kSlash + "Speedometer (MPH)", Math.round(velocity * 1000.0) / 1000.0);

    SmartDashboard.putNumber(SwerveConstants.kSlash + "Hub Distance",
        Math.round(getDistanceToHub(m_swerve.getPose()) * 1000.0) / 1000.0);

    // Driver tool: show the current hub-aim toggle state on the dashboard
    // so it's visible without digging through Shuffleboard/Elastic tabs.
    SmartDashboard.putBoolean(SwerveConstants.kSlash + "Hub Aim Enabled", UserConfig.getHubAimEnabled());

    // Driver tool: show whether the alliance hub is currently active per
    // the 2026 REBUILT shift schedule — scoring into an inactive hub is
    // worth 0 points, so this tells the driver when to actually shoot.
    SmartDashboard.putBoolean(SwerveConstants.kSlash + "Hub Active", isHubActive());

    // Store hub aim status periodically for use in commands
    m_isAimedAtHub = m_aimController.getError() < 10;

    m_robotPose = m_swerve.getPose();

    double[] fuel = SmartDashboard.getEntry("Fuel").getDoubleArray(new double[0]);
    Pose2d robotPose = m_swerve.getPose();

    int count = fuel.length / 3;
    double[] arr = new double[count * 3];

    for (int i = 0; i < count; i++) {

      double xRobot = fuel[i * 3 + 1];
      double yRobot = fuel[i * 3 + 2];

      Transform2d robotToFuel = new Transform2d(
          new Translation2d(xRobot, yRobot),
          new Rotation2d());

      Pose2d fuelFieldPose = robotPose.transformBy(robotToFuel);

      int base = i * 3;
      arr[base] = fuelFieldPose.getX();
      arr[base + 1] = fuelFieldPose.getY();
      arr[base + 2] = 0.0; // On the floor
    }

    SmartDashboard.getEntry("Field/Fuel").setDoubleArray(arr);

    // Limelight localization
    if (UserConfig.getAprilTagLocalizationEnabled()) {
      final String[] limelights = { "limelight-better" };

      for (String ll : limelights) {
        // MegaTag1
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);

        if ((mt1.tagCount == 1 && mt1.rawFiducials.length == 1 && mt1.rawFiducials[0].ambiguity <= 0.7
            && mt1.rawFiducials[0].distToCamera <= 3) ||
            (mt1.tagCount >= 2)) {
          m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 2));
          m_swerve.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }

        // MegaTag2
        LimelightHelpers.SetRobotOrientation(ll,
            m_swerve.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

        if (Math.abs(m_swerve.getGyro().getYawAngularVelocity().magnitude()) <= 360 && mt2.tagCount > 0) {
          m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE));
          // BUG FIX: this was adding mt1's pose/timestamp again instead of
          // mt2's. MegaTag2 estimates were computed but never actually used —
          // every vision update was MegaTag1 regardless of which branch ran.
          m_swerve.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
      }
    }
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          m_swerve::getPose,
          m_swerve::resetOdometry,
          m_swerve::getRobotVelocity,
          (speeds, feedforwards) -> m_swerve.drive(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5, 0, 0),
              new PIDConstants(5, 0, 0)),
          config,
          this::isRedAlliance,
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_swerve.field.getObject("path").setPoses(poses);
    });
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        m_swerve.getMaximumChassisVelocity(), 1.0,
        m_swerve.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        MetersPerSecond.of(0));
  }

  public Command drive(Supplier<ChassisSpeeds> velocity, Supplier<Boolean> hubAimActive) {
    return run(() -> {
      DriveMode driveMode = UserConfig.getDriveMode();

      ChassisSpeeds speeds = velocity.get();

      if (UserConfig.getHubAimEnabled() && hubAimActive.get()) {
        speeds = applyHubAim(speeds);
      }

      if (UserConfig.getBumpAimEnabled()) {
        speeds = applyBumpAim(speeds);
      }

      if (driveMode == DriveMode.RobotOriented) {
        m_swerve.drive(speeds);
      } else {
        m_swerve.driveFieldOriented(speeds);
      }
    });
  }

  private ChassisSpeeds applyHubAim(ChassisSpeeds speeds) {
    Translation2d robotTranslation = m_swerve.getPose().getTranslation();
    Translation2d targetTranslation = isRedAlliance() ? new Translation2d(11.917, 4.030)
        : new Translation2d(4.623, 4.030);

    Rotation2d angle = targetTranslation.minus(robotTranslation).getAngle();

    if (robotTranslation.getX() < 4 || robotTranslation.getX() > 12.5) {
      return new ChassisSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          m_aimController.calculate(m_swerve.getOdometryHeading().getDegrees(),
              angle.getDegrees()));
    } else {
      return speeds;
    }
  }

  private ChassisSpeeds applyBumpAim(ChassisSpeeds speeds) {

    // Each bump zone: {minX, minY, maxX, maxY}
    double[][] bumpZones = {
        { 3.5, 4.5, 5.75, 6.5 },
        { 3.5, 1.5, 5.75, 3.5 },
        { 10.75, 1.5, 13, 3.5 },
        { 10.75, 4.5, 13, 6.5 }
    };

    double robotX = m_swerve.getPose().getX();
    double robotY = m_swerve.getPose().getY();

    boolean inBumpZone = false;

    for (double[] zone : bumpZones) {
      if (robotX >= zone[0] && robotX <= zone[2] &&
          robotY >= zone[1] && robotY <= zone[3]) {
        inBumpZone = true;
        break;
      }
    }

    if (inBumpZone) {

      double currentHeading = m_swerve.getOdometryHeading().getDegrees();

      // Robot dimensions
      double length = 0.514;
      double width = 0.616;

      // Base diagonal angle (~39.84°)
      double baseAngle = Math.toDegrees(Math.atan(length / width));

      double[] snapAngles = {
          baseAngle,
          baseAngle + 90,
          baseAngle + 180,
          baseAngle + 270
      };

      // Find closest snap angle
      double closestAngle = snapAngles[0];
      double smallestError = Math.abs(MathUtil.inputModulus(currentHeading - closestAngle, -180, 180));

      for (double angle : snapAngles) {
        double error = Math.abs(MathUtil.inputModulus(currentHeading - angle, -180, 180));
        if (error < smallestError) {
          smallestError = error;
          closestAngle = angle;
        }
      }

      return new ChassisSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          m_aimController.calculate(currentHeading, closestAngle));

    } else {
      return speeds;
    }
  }

  public Command zeroGyroWithAllianceCommand() {
    return runOnce(() -> {
      DriverStation.getAlliance().ifPresent(alliance -> {
        m_swerve.zeroGyro();
        if (alliance == DriverStation.Alliance.Red) {
          m_swerve.resetOdometry(new Pose2d(m_swerve.getPose().getTranslation(), Rotation2d.k180deg));
        }
      });
    });
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerve;
  }

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public static boolean isAimedAtHub() {
    return m_isAimedAtHub;
  }

  public static double getDistanceToHub(Pose2d robotPose) {
    Translation2d targetTranslation = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
            ? new Translation2d(11.917, 4.030)
            : new Translation2d(4.623, 4.030);

    return robotPose.getTranslation().getDistance(targetTranslation);
  }

  /**
   * 2026 REBUILT hub active/inactive shift logic — fuel scored into an
   * inactive hub is worth 0 points. This follows WPILib's official
   * reference implementation (docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html):
   * the field picks which alliance's hub goes inactive first (sent via
   * DriverStation.getGameSpecificMessage(), 'R' or 'B'), then hubs alternate
   * active/inactive through four ~25s shifts. The hub is always active
   * during auto, the 10s transition shift, and the final 30s of teleop.
   */
  public static boolean isHubActive() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      // No alliance assigned (e.g. not connected to FMS/DS yet) — no hub.
      return false;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      // Disabled or between periods — no active hub to speak of.
      return false;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      // Likely just entered teleop and the field hasn't sent it yet.
      return true;
    }

    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R':
        redInactiveFirst = true;
        break;
      case 'B':
        redInactiveFirst = false;
        break;
      default:
        // Corrupt/unexpected data — assume active rather than sitting idle.
        return true;
    }

    boolean isRed = alliance.get() == DriverStation.Alliance.Red;
    boolean shift1Active = isRed ? !redInactiveFirst : redInactiveFirst;

    if (matchTime > 130) {
      return true; // Transition shift (2:20-2:10 remaining) — always active
    } else if (matchTime > 105) {
      return shift1Active; // Shift 1 (2:10-1:45)
    } else if (matchTime > 80) {
      return !shift1Active; // Shift 2 (1:45-1:20)
    } else if (matchTime > 55) {
      return shift1Active; // Shift 3 (1:20-0:55)
    } else if (matchTime > 30) {
      return !shift1Active; // Shift 4 (0:55-0:30)
    } else {
      return true; // Endgame (final 30s) — always active
    }
  }

  public void zeroGyro() {
    m_swerve.zeroGyro();
  }
}