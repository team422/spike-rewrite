package frc.robot;

import edu.wpi.first.hal.HALUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
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

  // TODO: Periodics
  public void alignShootingPeriodic() {}

  public void autoShootingPeriodic() {}

  public void ampPeriodic() {}

  public void updateAction(RobotAction action) {
    DriveProfiles newDriveProfile = m_drive.getCurrentProfile();
    IndexerState newIndexerState = m_indexer.getIndexerState();
    IntakeState newIntakeState = m_intake.getIntakeState();
    ShooterState newShooterState = m_shooter.getShooterState();

    switch (action) {
      case kTeleopDefault:
      case kAutoDefault:
        newDriveProfile = DriveProfiles.kDefault;
        newIndexerState = IndexerState.kIdle;
        newIntakeState = IntakeState.kIdle;
        newShooterState = ShooterState.kIdle;
        break;
      case kAlignShooting:
      case kAutoShooting:
        newDriveProfile = DriveProfiles.kAutoAlign;
      case kSubwooferShooting:
        newShooterState = ShooterState.kRevving;
        newIntakeState = IntakeState.kIdle;
        break;
      case kIntake:
        newDriveProfile = DriveProfiles.kDefault;
        newIntakeState = IntakeState.kIntaking;
        newIndexerState = IndexerState.kIntaking;
        newShooterState = ShooterState.kIdle;
        break;
      case kVomitting:
        newDriveProfile = DriveProfiles.kDefault;
        newIndexerState = IndexerState.kVomitting;
        newIntakeState = IntakeState.kVomit;
        newShooterState = ShooterState.kRejecting;
        break;
      case kAmp:
        newDriveProfile = DriveProfiles.kDefault;
        newIntakeState = IntakeState.kIdle;
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
}
