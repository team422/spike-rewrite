package frc.robot;

import edu.wpi.first.hal.HALUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  // Subsystems
  private Drive m_drive;

  // TODO: do later gng
  public static enum RobotAction {
    kTeleopDefault,

    kAutoDefault,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  private static RobotState m_instance;

  public static void startInstance(Drive drive) {
    m_instance = new RobotState(drive);
  }

  private RobotState(Drive drive) {
    m_drive = drive;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();

    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kAutoDefault, () -> {});

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

  public void updateAction(RobotAction action) {
    // TODO: with more subsystems comes more responsibility
    DriveProfiles driveProfiles = m_drive.getDefaultProfile();

    switch (action) {
      case kTeleopDefault:
      case kAutoDefault:
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
