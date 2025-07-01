package frc.robot;

import edu.wpi.first.hal.HALUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
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

  // TODO: do later gng
  public static enum RobotAction {
    kTeleopDefault,
    kIntake,
    kRevving,
    kVomitting,
    kAlignShooting,
    kSubwooferShooting,

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
    periodicHash.put(RobotAction.kRevving, this::revvingPeriodic);
    periodicHash.put(RobotAction.kVomitting, () -> {});
    periodicHash.put(RobotAction.kAlignShooting, this::alignShootingPeriodic);
    periodicHash.put(RobotAction.kSubwooferShooting, () -> {});

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

  public void revvingPeriodic(){

  }

  public void alignShootingPeriodic(){

  }

  public void autoShootingPeriodic(){

  }

  public void updateAction(RobotAction action) {
    // TODO: with more subsystems comes more responsibility
    DriveProfiles driveProfiles = m_drive.getDefaultProfile();

    switch (action) {
      case kTeleopDefault:
      case kAutoDefault:
        break;
      case kAlignShooting:
        break;
      case kAutoShooting:
        break;
      case kIntake:
        break;
      case kRevving:
        break;
      case kSubwooferShooting:
        break;
      case kVomitting:
        break;
    }

    m_drive.updateProfile(driveProfiles);
  }

  public void setDefaultAction() {
    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      updateAction(RobotAction.kAutoDefault);
    } else {
      updateAction(RobotAction.kTeleopDefault);
    }
  }
}
