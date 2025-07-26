package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.lib.littletonUtils.SwerveSetpointGenerator.ModuleLimits;

public class Constants {
  public static enum Mode {
    REAL,
    SIM,
    REPLAY,
  }

  public static final Mode kRealMode = Mode.REAL;
  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? kRealMode : kSimMode;

  public static final boolean kTuningMode = true;

  public static final class DriveConstants {
    public static final double kMaxLinearSpeed = 4.5; // meters per second
    // Tommy - this constant is unused right now, see my comment in driveToPointPeriodic
    public static final double kMaxDriveToPointSpeed = 3.6;
    // Tommy - delete these two constants after resolving the comment above
    public static final double kMaxAutoscoreSpeed = 3.6;
    public static final double kMaxAutoIntakeSpeed = 4.0;
    public static final double kMaxMeshedSpeed = 4.5;
    public static final double kMaxLinearAcceleration = 3.0; // meters per second squared
    public static final double kTrackWidthX = Units.inchesToMeters(22.75);
    public static final double kTrackWidthY = Units.inchesToMeters(22.75);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;
    public static final double kMaxAngularAcceleration = kMaxLinearAcceleration / kDriveBaseRadius;
    public static final LoggedTunableNumber kTeleopRotationSpeed =
        new LoggedTunableNumber("Teleop Rotation Speed", 10.0);
    public static final double kVisionSpeedConstantK = 0.5;

    // the exponent to raise the input to
    // ex 1.0 is linear, 2.0 is squared, etc
    public static final LoggedTunableNumber kDriveControlsExponent =
        new LoggedTunableNumber("Drive Control Mode", 2.0);

    public static final Translation2d[] kModuleTranslations =
        new Translation2d[] {
          new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
        };

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final ModuleLimits kModuleLimitsFree =
        new ModuleLimits(kMaxLinearSpeed, kMaxAngularSpeed, Units.degreesToRadians(1080.0));

    public static final double kWheelRadius = Units.inchesToMeters(1.935);
    public static final double kOdometryFrequency = 250.0;

    public static final double kDriveGearRatio =
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // L2 gear ratio
    public static final double kTurnGearRatio = 150.0 / 7.0;

    public static final double kAutoAlignTolerance = 2.0;

    // Simulation constants
    public static final double kDriveSimGearRatio = kDriveGearRatio;
    public static final double kDriveSimMOI = 0.025;
    public static final double kTurnSimGearRatio = kTurnGearRatio;
    public static final double kTurnSimMOI = 0.004;

    // universal reversals for drive (aka the big negative sign)
    // Tommy - if you find yourself needing to enable either of these, most likely your controls are
    // wrong or your modules are ID'd wrong
    public static final boolean kRealReversed = false;
    public static final boolean kSimReversed = false;

    public static final LoggedTunableNumber kDriveToPointP =
        new LoggedTunableNumber("DriveToPoint P", 3.2);
    public static final LoggedTunableNumber kDriveToPointI =
        new LoggedTunableNumber("DriveToPoint I", 0.0);
    public static final LoggedTunableNumber kDriveToPointD =
        new LoggedTunableNumber("DriveToPoint D", 0.18);

    public static final LoggedTunableNumber kDriveToPointAutoP =
        new LoggedTunableNumber("DriveToPoint Auto P", 3.0);
    public static final LoggedTunableNumber kDriveToPointAutoI =
        new LoggedTunableNumber("DriveToPoint Auto I", 0.0);
    public static final LoggedTunableNumber kDriveToPointAutoD =
        new LoggedTunableNumber("DriveToPoint Auto D", 0.12);

    public static final LoggedTunableNumber kDriveToPointHeadingP =
        new LoggedTunableNumber("DriveToPoint Heading P", 4.0);
    public static final LoggedTunableNumber kDriveToPointHeadingI =
        new LoggedTunableNumber("DriveToPoint Heading I", 0.0);
    public static final LoggedTunableNumber kDriveToPointHeadingD =
        new LoggedTunableNumber("DriveToPoint Heading D", 0.05);

    public static final LoggedTunableNumber kDriveToPointMaxVelocity =
        new LoggedTunableNumber("DriveToPoint Max Velocity", 3.0);
    public static final LoggedTunableNumber kDriveToPointMaxAcceleration =
        new LoggedTunableNumber("DriveToPoint Max Acceleration", 4.0);
    public static final LoggedTunableNumber kDriveToPointMaxDeceleration =
        new LoggedTunableNumber("DriveToPoint Max Deceleration", 3.0);

    public static final LoggedTunableNumber kMeshedXYP =
        new LoggedTunableNumber("Drive Meshed XY P", 3.0);
    public static final LoggedTunableNumber kMeshedXYD =
        new LoggedTunableNumber("Drive Meshed XY D", 0.12);
    public static final LoggedTunableNumber kMeshedThetaP =
        new LoggedTunableNumber("Drive Meshed Theta P", 3.0);
    public static final LoggedTunableNumber kMeshedThetaD =
        new LoggedTunableNumber("Drive Meshed Theta D", 0.0);
    public static final LoggedTunableNumber kDebounceAmount =
        new LoggedTunableNumber("Meshed Drive Debounce", 0.1);
    public static final LoggedTunableNumber kMeshDrivePriority =
        new LoggedTunableNumber("Meshed Drive Priority", 0.3);

    public static final LoggedTunableNumber kAutoscoreDeployDistance =
        new LoggedTunableNumber("Autoscore Deploy Distance", 42.0);
    public static final LoggedTunableNumber kAutoscoreOuttakeDistance =
        new LoggedTunableNumber("Autoscore Outtake Distance", 1.25);
    public static final LoggedTunableNumber kAutoscoreL1OuttakeDistance =
        new LoggedTunableNumber("Autoscore L1 Outtake Distance", 18.0);
    public static final LoggedTunableNumber kBargeScoreThrowDistance =
        new LoggedTunableNumber("Barge Score Throw Distance", 10.0);
    public static final LoggedTunableNumber kLoaderStationTimeout =
        new LoggedTunableNumber("Loader Station Timeout", 0.25);

    // radians per second squared to be considered slipping
    public static final LoggedTunableNumber kSlipThreshold =
        new LoggedTunableNumber("Slip Threshold", 150000);

    // meters per second squared to be considered in freefall (less than)
    public static final double kFreefallAccelerationThreshold = 9.0;

    public static final Mass kRobotMass = Pounds.of(120);
    public static final MomentOfInertia kRobotMOI = KilogramSquareMeters.of(6.5);
  }

  public static final class ShooterConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("Shooter/idleVoltage", 0.0);
    public static final LoggedTunableNumber kTopRPS =
        new LoggedTunableNumber("Shooter/topRPS", 0.0);
    public static final LoggedTunableNumber kBottomRPS =
        new LoggedTunableNumber("Shooter/bottomRPS", 0.0);
    public static final LoggedTunableNumber kRejectingVoltage =
        new LoggedTunableNumber("Shooter/rejectingVoltage", 0.0);
    public static final LoggedTunableNumber kTopAmpVelocity =
        new LoggedTunableNumber("Shooter/topAmpVelocity", 0);
    public static final LoggedTunableNumber kBottomAmpVelocity =
        new LoggedTunableNumber("Shooter/bottomAmpVelocity", 0);

    public static final LoggedTunableNumber kTopShooterP =
        new LoggedTunableNumber("Shooter/TopP", 0.0);
    public static final LoggedTunableNumber kTopShooterI =
        new LoggedTunableNumber("Shooter/TopI", 0.0);
    public static final LoggedTunableNumber kTopShooterD =
        new LoggedTunableNumber("Shooter/TopD", 0.0);
    public static final LoggedTunableNumber kBottomShooterP =
        new LoggedTunableNumber("Shooter/BottomP", 0.0);
    public static final LoggedTunableNumber kBottomShooterI =
        new LoggedTunableNumber("Shooter/BottomI", 0.0);
    public static final LoggedTunableNumber kBottomShooterD =
        new LoggedTunableNumber("Shooter/BottomD", 0.0);

    public static final LoggedTunableNumber kTopKs = new LoggedTunableNumber("Shooter/TopKs", 0.0);
    public static final LoggedTunableNumber kTopKv = new LoggedTunableNumber("Shooter/TopKv", 0.0);
    public static final LoggedTunableNumber kBottomKs =
        new LoggedTunableNumber("Shooter/BottomKs", 0.0);
    public static final LoggedTunableNumber kBottomKv =
        new LoggedTunableNumber("Shooter/BottomKv", 0.0);

    public static final double kVelocityTolerance = 6.0; // in RPM

    // sim
    public static final DCMotor kTopDCMotor = DCMotor.getNEO(1);
    public static final DCMotor kBottomDCMotor = DCMotor.getNEO(1);
    public static final double kSimMOI = 10.0;
    public static final double kSimGearing = 10.0;

    public static final LoggedTunableNumber kSimTopShooterP =
        new LoggedTunableNumber("Shooter/SimTopP", 0.0);
    public static final LoggedTunableNumber kSimTopShooterI =
        new LoggedTunableNumber("Shooter/SimTopI", 0.0);
    public static final LoggedTunableNumber kSimTopShooterD =
        new LoggedTunableNumber("Shooter/SimTopD", 0.0);
    public static final LoggedTunableNumber kSimBottomShooterP =
        new LoggedTunableNumber("Shooter/SimBottomP", 0.0);
    public static final LoggedTunableNumber kSimBottomShooterI =
        new LoggedTunableNumber("Shooter/SimBottomI", 0.0);
    public static final LoggedTunableNumber kSimBottomShooterD =
        new LoggedTunableNumber("Shooter/SimBottomD", 0.0);

    public static final LoggedTunableNumber kSimTopKs =
        new LoggedTunableNumber("Shooter/SimTopKs", 0.0);
    public static final LoggedTunableNumber kSimTopKv =
        new LoggedTunableNumber("Shooter/SimTopKv", 0.0);
    public static final LoggedTunableNumber kSimBottomKs =
        new LoggedTunableNumber("Shooter/SimBottomKs", 0.0);
    public static final LoggedTunableNumber kSimBottomKv =
        new LoggedTunableNumber("Shooter/SimBottomKv", 0.0);
  }

  public static final class IndexerConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("Indexer/idleVoltage", 0.0);
    public static final LoggedTunableNumber kIntakingVoltage =
        new LoggedTunableNumber("Indexer/intakingVoltage", 0.0);
    public static final LoggedTunableNumber kShootingVoltage =
        new LoggedTunableNumber("Indexer/shootingVoltage", 0.0);
    public static final LoggedTunableNumber kVomitVoltage =
        new LoggedTunableNumber("Indexer/vomitVoltage", 0.0);

    // sim
    public static final DCMotor kDCMotor = DCMotor.getNEO(1);
    public static final double kSimMOI = 10.0;
    public static final double kSimGearing = 10.0;
  }

  public static final class IntakeConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("Intake/idleVoltage", 0.0);
    public static final LoggedTunableNumber kIntakingVoltage =
        new LoggedTunableNumber("Intake/intakingVoltage", 0.0);
    public static final LoggedTunableNumber kVomitVoltage =
        new LoggedTunableNumber("Intake/vomitVoltage", 0.0);

    // sim
    public static final DCMotor kDCMotor = DCMotor.getNEO(1);
    public static final double kSimMOI = 10.0;
    public static final double kSimGearing = 10.0;
  }

  public static final class Ports {
    public static final int kFrontLeftDrive = 0;
    public static final int kFrontLeftTurn = 1;
    public static final int kFrontLeftCancoder = 2;

    public static final int kFrontRightDrive = 3;
    public static final int kFrontRightTurn = 4;
    public static final int kFrontRightCancoder = 5;

    public static final int kBackLeftDrive = 6;
    public static final int kBackLeftTurn = 7;
    public static final int kBackLeftCancoder = 8;

    public static final int kBackRightDrive = 9;
    public static final int kBackRightTurn = 10;
    public static final int kBackRightCancoder = 11;

    public static final int kIndexer = 12;
    public static final int kPhotoelectric1 = 13;
    public static final int kPhotoelectric2 = 14;

    public static final int kIntake = 15;

    public static final int kTopShooter = 16;
    public static final int kBottomShooter = 17;

    public static final int kPigeon = 22;

    public static final String kDriveCanivoreName = "Drivetrain";

    public static final String kMainCanivoreName = "Main";
  }

  public static final class CurrentLimitConstants {
    // Drive
    public static final double kDriveDefaultSupplyCurrentLimit = 75.0;
    public static final double kDriveDefaultStatorCurrentLimit = 180.0;

    public static final double kTurnDefaultSupplyCurrentLimit = 60.0;
    public static final double kTurnDefaultStatorCurrentLimit = 120.0;
  }

  public static final class FieldConstants {
    // copied from frc-24 with some minor tweaks

    public static final Translation2d kSource = new Translation2d(15.696, 0.701);

    public static final Translation3d kTopRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(0.0), Units.inchesToMeters(238.815), Units.inchesToMeters(83.091));

    public static final Translation3d kTopLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d kBottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d kBottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d kCenterSpeakerOpening =
        kBottomLeftSpeaker.interpolate(kTopRightSpeaker, 0.5);

    public static final double kFieldLength = Units.inchesToMeters(651.223);
    public static final double kFieldWidth = Units.inchesToMeters(323.277);
    public static final double kWingX = Units.inchesToMeters(229.201);
    public static final double kPodiumX = Units.inchesToMeters(126.75);
    public static final double kStartingLineX = Units.inchesToMeters(74.111);

    public static final double kAprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static final Pose2d kAmpBlue = new Pose2d(1.749, 7.82, Rotation2d.fromDegrees(90));
    public static final Pose2d kDailedShot = new Pose2d(2.95, 4.08, new Rotation2d(145.00));
    public static final Translation2d kCorner = new Translation2d(0, 7.82);
    public static final Translation2d kFeederAim = new Translation2d(1, 6.82);
    public static final Translation2d kSourceMidShot = new Translation2d(8.04, kFieldWidth - 2);

    // halfway between midline and wing
    public static final Translation2d kFeedingSetpoint = new Translation2d(7.137, 0.872);
  }
}
