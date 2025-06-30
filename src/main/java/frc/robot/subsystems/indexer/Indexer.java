package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO m_io;
  private SubsystemProfiles<IndexerState> m_profiles;
  private IndexerInputsAutoLogged m_inputs;

  public enum IndexerState {
    kIdle,
    kIntaking,
    kIndexing,
    kReversing,
    kShooting,
    kVomitting
  }

  public Indexer(IndexerIO io) {
    m_io = io;
    m_inputs = new IndexerInputsAutoLogged();

    HashMap<IndexerState, Runnable> periodics = new HashMap<>();
    periodics.put(IndexerState.kIdle, this::idlePeriodic);
    periodics.put(IndexerState.kIntaking, this::intakingPeriodic);
    periodics.put(IndexerState.kIndexing, this::indexingPeriodic);
    periodics.put(IndexerState.kReversing, this::reversingPeriodic);
    periodics.put(IndexerState.kShooting, this::shootingPeriodic);
    periodics.put(IndexerState.kVomitting, this::vomitPeriodic);

    m_profiles = new SubsystemProfiles<>(periodics, IndexerState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Indexer/inputs", m_inputs);
  }

  public void idlePeriodic() {
    m_io.setVoltage(IndexerConstants.kIdleVoltage);
  }

  public void intakingPeriodic() {
    if (!m_io.hasPiece()) {
      m_io.setVoltage(IndexerConstants.kIntakingVoltage);
    } else {
      updateState(IndexerState.kIndexing);
      indexingPeriodic();
    }
  }

  public void indexingPeriodic() {
    m_io.setVoltage(IndexerConstants.kIndexingVoltage);
  }

  public void reversingPeriodic() {
    m_io.setVoltage(IndexerConstants.kReversingVoltage);
  }

  public void shootingPeriodic() {
    m_io.setVoltage(IndexerConstants.kShootingVoltage);
  }

  public void vomitPeriodic() {
    m_io.setVoltage(IndexerConstants.kVomitVoltage);
  }

  public void updateState(IndexerState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(IndexerConstants.kIdleVoltage);
      case kIndexing:
      case kIntaking:
      case kReversing:
      case kShooting:
      case kVomitting:
        break;
    }
    m_profiles.setCurrentProfile(state);
  }
}
