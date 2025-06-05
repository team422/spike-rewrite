package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public class FieldConstants {
    public static final double kFieldLength = Units.inchesToMeters(651.223);
    public static final double kFieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);

    public static final Translation2d ampCenter =
        new Translation2d(Units.inchesToMeters(72.455), kFieldWidth);

    /** Staging locations for each note */
    public static final class StagingLocations {
      public static final double centerlineX = kFieldLength / 2.0;

      // need to update
      public static final double centerlineFirstY = Units.inchesToMeters(29.638);
      public static final double centerlineSeparationY = Units.inchesToMeters(66);
      public static final double spikeX = Units.inchesToMeters(114);
      // need
      public static final double spikeFirstY = Units.inchesToMeters(161.638);
      public static final double spikeSeparationY = Units.inchesToMeters(57);

      public static final Translation2d[] centerlineTranslations = new Translation2d[5];
      public static final Translation2d[] spikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] =
              new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }
    }

    /** Each corner of the speaker * */
    public static final class Speaker {

      // corners (blue alliance origin)
      public static final Translation3d topRightSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(238.815),
              Units.inchesToMeters(83.091));

      public static final Translation3d topLeftSpeaker =
          new Translation3d(
              Units.inchesToMeters(18.055),
              Units.inchesToMeters(197.765),
              Units.inchesToMeters(83.091));

      public static final Translation3d bottomRightSpeaker =
          new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
      public static final Translation3d bottomLeftSpeaker =
          new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

      /** Center of the speaker opening (blue alliance) */
      public static final Translation3d centerSpeakerOpening =
          bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
    }

    public static final class Subwoofer {
      public static final Pose2d ampFaceCorner =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(239.366),
              Rotation2d.fromDegrees(-120));

      public static final Pose2d sourceFaceCorner =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(197.466),
              Rotation2d.fromDegrees(120));

      public static final Pose2d centerFace =
          new Pose2d(
              Units.inchesToMeters(35.775),
              Units.inchesToMeters(218.416),
              Rotation2d.fromDegrees(180));
    }

    public static final class Stage {
      public static final Pose2d podiumLeg =
          new Pose2d(Units.inchesToMeters(126.75), Units.inchesToMeters(161.638), new Rotation2d());
      public static final Pose2d ampLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(212.425),
              Rotation2d.fromDegrees(-30));
      public static final Pose2d sourceLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(110.837),
              Rotation2d.fromDegrees(30));

      public static final Pose2d centerPodiumAmpChain =
          new Pose2d(
              podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(120.0));
      public static final Pose2d centerAmpSourceChain =
          new Pose2d(
              ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5),
              new Rotation2d());
      public static final Pose2d centerSourcePodiumChain =
          new Pose2d(
              sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(240.0));
      public static final Pose2d center =
          new Pose2d(Units.inchesToMeters(192.55), Units.inchesToMeters(161.638), new Rotation2d());
      public static final double centerToChainDistance =
          center.getTranslation().getDistance(centerPodiumAmpChain.getTranslation());
    }

    public static final class Amp {
      public static final Translation2d ampTapeTopCorner =
          new Translation2d(Units.inchesToMeters(130.0), Units.inchesToMeters(305.256));
      public static final double ampBottomY = kFieldWidth - Units.inchesToMeters(17.75);
    }

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  }
}
