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
    kRevving,
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
    periodicHash.put(RobotAction.kRevving, this::revvingPeriodic);
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

  //the only thing im still confused on how to do are these periodics
  public void revvingPeriodic() {
   

  }

  public void alignShootingPeriodic() {

  }

  public void autoShootingPeriodic() {

  }

  public void ampPeriodic(){

  }

  public void updateAction(RobotAction action) {
    switch (action) {
      case kTeleopDefault:
      case kAutoDefault:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_indexer.updateState(IndexerState.kIdle);
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kIdle);
        break;
      case kAlignShooting:
      case kAutoShooting:
      case kSubwooferShooting:
      case kRevving:
        m_drive.updateProfile(DriveProfiles.kAutoAlign);
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kRevving);
        break;
      case kIntake:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.updateState(IntakeState.kIntaking);
        m_indexer.updateState(IndexerState.kIntaking);
        m_shooter.updateState(ShooterState.kIdle);
        break;
      case kVomitting:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_indexer.updateState(IndexerState.kVomitting);
        m_intake.updateState(IntakeState.kVomit);
        m_shooter.updateState(ShooterState.kRejecting);
        break;
      case kAmp:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.updateState(IntakeState.kIdle);
        m_shooter.updateState(ShooterState.kAmp);
        break;
    }

    m_profiles.setCurrentProfile(action);
  }

  public void setDefaultAction() {
    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      updateAction(RobotAction.kAutoDefault);
    } else {
      updateAction(RobotAction.kTeleopDefault);
    }
  }
}
