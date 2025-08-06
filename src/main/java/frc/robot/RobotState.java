package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.ShooterMath;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  // Subsystems
  private Drive m_drive;
  private Shooter m_shooter;
  private Indexer m_indexer;
  private Intake m_intake;

  public static enum RobotAction {
    kTeleopDefault,
    kIntake,
    kVomitting,
    kAlignShooting,
    kSubwooferShooting,
    kAmp,

    kAutoDefault,
    kAutoShooting,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  private static RobotState m_instance;

  public static void startInstance(Drive drive, Shooter shooter, Indexer indexer, Intake intake) {
    m_instance = new RobotState(drive, shooter, indexer, intake);
  }

  private RobotState(Drive drive, Shooter shooter, Indexer indexer, Intake intake) {
    m_drive = drive;
    m_shooter = shooter;
    m_indexer = indexer;
    m_intake = intake;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();

    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kIntake, () -> {});
    periodicHash.put(RobotAction.kVomitting, () -> {});
    periodicHash.put(RobotAction.kAlignShooting, this::alignShootingPeriodic);
    periodicHash.put(RobotAction.kSubwooferShooting, () -> {});
    periodicHash.put(RobotAction.kAmp, this::ampPeriodic);

    periodicHash.put(RobotAction.kAutoDefault, () -> {});
    periodicHash.put(RobotAction.kAutoShooting, this::autoShootingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTeleopDefault);
  }

  public static RobotState getInstance() {
    if (m_instance == null) {
      throw new IllegalStateException("Yo call startInstance before using RobotState gng");
    }
    return m_instance;
  }

  public void onEnable() {}

  public void onDisable() {}

  public void updateRobotState() {
    long start = HALUtil.getFPGATime();

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.recordOutput("PeriodicTime/RobotState", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void alignShootingPeriodic() {
    var speeds = ShooterMath.calculateSpeakerFlywheelSpeed(m_drive.getPose());
    m_shooter.setVelocity(speeds.getFirst(), speeds.getSecond());
    m_drive.setDesiredHeading(ShooterMath.calculateSpeakerHeading(m_drive.getPose()));
  }

  public void autoShootingPeriodic() {
    var speeds = ShooterMath.calculateSpeakerFlywheelSpeed(m_drive.getPose());
    m_shooter.setVelocity(speeds.getFirst(), speeds.getSecond());
    var rot = ShooterMath.calculateSpeakerHeading(m_drive.getPose());
    m_drive.setDesiredHeading(rot);

    boolean driveWithinTolerance =
        rot.minus(m_drive.getRotation()).getMeasure().abs(Degrees)
            < DriveConstants.kAutoAlignTolerance;
    Logger.recordOutput(
        "Indexer/withinTolerance", driveWithinTolerance && m_shooter.withinTolerance());
    if (driveWithinTolerance && m_shooter.withinTolerance()) {
      m_indexer.updateState(IndexerState.kShooting);
    }
  }

  public void ampPeriodic() {
    m_drive.setDesiredHeading(Rotation2d.fromDegrees(90));
  }

  public void updateAction(RobotAction action) {
    DriveProfiles newDriveProfile = DriveProfiles.kDefault;
    IndexerState newIndexerState = IndexerState.kIdle;
    IntakeState newIntakeState = IntakeState.kIdle;
    ShooterState newShooterState = ShooterState.kIdle;

    switch (action) {
      case kTeleopDefault:
      case kAutoDefault:
        break;
      case kAlignShooting:
      case kAutoShooting:
        newDriveProfile = DriveProfiles.kAutoAlign;
        newShooterState = ShooterState.kRevving;
        break;

      case kSubwooferShooting:
        newShooterState = ShooterState.kRevving;
        break;

      case kIntake:
        newIntakeState = IntakeState.kIntaking;
        newIndexerState = IndexerState.kIntaking;
        break;

      case kVomitting:
        newIndexerState = IndexerState.kVomitting;
        newIntakeState = IntakeState.kVomit;
        newShooterState = ShooterState.kRejecting;
        break;

      case kAmp:
        newDriveProfile = DriveProfiles.kAutoAlign;
        newShooterState = ShooterState.kAmp;
        break;
    }

    m_profiles.setCurrentProfile(action);

    if (newDriveProfile != m_drive.getCurrentProfile()) {
      m_drive.updateProfile(newDriveProfile);
    }
    if (newIndexerState != m_indexer.getIndexerState()) {
      m_indexer.updateState(newIndexerState);
    }
    if (newIntakeState != m_intake.getIntakeState()) {
      m_intake.updateState(newIntakeState);
    }
    if (newShooterState != m_shooter.getShooterState()) {
      m_shooter.updateState(newShooterState);
    }
  }

  public void setDefaultAction() {
    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      updateAction(RobotAction.kAutoDefault);
    } else {
      updateAction(RobotAction.kTeleopDefault);
    }
  }

  public RobotAction getAction(){
    return m_profiles.getCurrentProfile();
  }
}
