package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO m_io;
  private IntakeInputsAutoLogged m_inputs;
  private SubsystemProfiles<IntakeState> m_profiles;

  public enum IntakeState {
    kIdle,
    kIntaking,
    kVomit
  }

  public Intake(IntakeIO io) {
    m_io = io;
    m_inputs = new IntakeInputsAutoLogged();

    HashMap<IntakeState, Runnable> periodics = new HashMap<>();

    periodics.put(IntakeState.kIdle, this::idlePeriodic);
    periodics.put(IntakeState.kIntaking, this::intakingPeriodic);
    periodics.put(IntakeState.kVomit, this::vomitPeriodic);
    m_profiles = new SubsystemProfiles<Intake.IntakeState>(periodics, IntakeState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Intake/inputs", m_inputs);
  }

  public void idlePeriodic() {
    m_io.setVoltage(IntakeConstants.kIdleVoltage);
  }

  public void intakingPeriodic() {
    m_io.setVoltage(IntakeConstants.kIntakingVoltage);
  }

  public void vomitPeriodic() {
    m_io.setVoltage(IntakeConstants.kVomitVoltage);
  }

  public void updateState(IntakeState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(IntakeConstants.kIdleVoltage);
      case kIntaking:
      case kVomit:
        break;
    }
    m_profiles.setCurrentProfile(state);
  }
}
