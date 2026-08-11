package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class UserConfig {

    public enum DriveMode {
        RobotOriented,
        FieldOrientedAngularVelocity,
        FieldOrientedDirectAngle
    }

    private static final SendableChooser<DriveMode> m_driveModeChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_hubAimChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_bumpAimChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_beansModeChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_apriltagLocalizationChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_manualShooterRPMChooser = new SendableChooser<>();

    // Quick in-match toggle for hub-aim, flipped from a controller button.
    // SendableChooser has no programmatic "set selected" — only the
    // dashboard widget can write to it — so this is a separate flag that
    // ANDs with the chooser below rather than trying to drive the chooser
    // itself. Leave the dashboard chooser on "Enabled" and this becomes the
    // real on/off switch during a match.
    private static boolean m_hubAimButtonOverride = true;

    public static final void initialize() {
        // Direct-angle heading control was removed from RobotContainer (too
        // twitchy for manual driving) — angular-velocity is now the only
        // field-oriented rotation mode, and the default.
        m_driveModeChooser.setDefaultOption("Field-Oriented Angular Velocity", DriveMode.FieldOrientedAngularVelocity);
        m_driveModeChooser.addOption("Robot-Oriented", DriveMode.RobotOriented);

        m_hubAimChooser.setDefaultOption("Enabled", true);
        m_hubAimChooser.addOption("Disabled", false);

        m_bumpAimChooser.setDefaultOption("Enabled", true);
        m_bumpAimChooser.addOption("Disabled", false);

        m_beansModeChooser.setDefaultOption("Enabled", true);
        m_beansModeChooser.addOption("Disabled", false);

        m_apriltagLocalizationChooser.setDefaultOption("Enabled", true);
        m_apriltagLocalizationChooser.addOption("Disabled", false);

        // Off the competition field, distance-to-hub is meaningless (no
        // AprilTags to correct pose), so the auto RPM table always lands on
        // whatever distance the robot's pose happens to be stuck at. This
        // lets the existing "Shooter RPM Tuner" number actually drive the
        // shooter directly for bench testing instead of being unused.
        m_manualShooterRPMChooser.setDefaultOption("Disabled", false);
        m_manualShooterRPMChooser.addOption("Enabled", true);

        SmartDashboard.putData("Drive Mode", m_driveModeChooser);
        SmartDashboard.putData("Hub Aim", m_hubAimChooser);
        SmartDashboard.putData("Bump Aim", m_bumpAimChooser);
        SmartDashboard.putData("Beans Mode", m_beansModeChooser);
        SmartDashboard.putData("AprilTag Localization", m_apriltagLocalizationChooser);
        SmartDashboard.putData("Manual Shooter RPM", m_manualShooterRPMChooser);
        SmartDashboard.putNumber("Shooter RPM Tuner", ShooterConstants.kShooterRPM);
    }

    public static DriveMode getDriveMode() {
        return m_driveModeChooser.getSelected();
    }

    public static boolean getHubAimEnabled() {
        return m_hubAimChooser.getSelected() && m_hubAimButtonOverride;
    }

    /** Flips the button-controlled hub-aim override. Bound to an operator button in RobotContainer. */
    public static void toggleHubAim() {
        m_hubAimButtonOverride = !m_hubAimButtonOverride;
    }

    public static boolean getBumpAimEnabled() {
        return m_bumpAimChooser.getSelected();
    }

    public static boolean getBeansModeEnabled() {
        return m_beansModeChooser.getSelected();
    }

    public static boolean getAprilTagLocalizationEnabled() {
        return m_apriltagLocalizationChooser.getSelected();
    }

    public static boolean getManualShooterRPMEnabled() {
        return m_manualShooterRPMChooser.getSelected();
    }

    public static double getShooterRPM() {
        return SmartDashboard.getNumber("Shooter RPM Tuner", ShooterConstants.kShooterRPM);
    }
}