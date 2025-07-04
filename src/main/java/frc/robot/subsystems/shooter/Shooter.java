package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO m_io;
  private ShooterInputsAutoLogged m_inputs;
  private SubsystemProfiles<ShooterState> m_profiles;
  private PIDController m_topController;
  private PIDController m_bottomController;
  private SimpleMotorFeedforward m_topFF;
  private SimpleMotorFeedforward m_bottomFF;

  public enum ShooterState {
    kIdle,
    kRevving,
    kRejecting,
    kAmp
  }

  public Shooter(
      ShooterIO shooter,
      PIDController topController,
      PIDController bottomController,
      SimpleMotorFeedforward topFF,
      SimpleMotorFeedforward bottomFF) {
    m_io = shooter;
    m_inputs = new ShooterInputsAutoLogged();
    m_topController = topController;
    m_bottomController = bottomController;
    m_topFF = topFF;
    m_bottomFF = bottomFF;

    HashMap<ShooterState, Runnable> periodics = new HashMap<>();

    periodics.put(ShooterState.kIdle, this::idlePeriodic);
    periodics.put(ShooterState.kRevving, this::revvingPeriodic);
    periodics.put(ShooterState.kRejecting, this::rejectingPeriodic);
    periodics.put(ShooterState.kAmp, this::ampPeriodic);

    m_profiles = new SubsystemProfiles<>(periodics, ShooterState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Shooter/inputs", m_inputs);
    Logger.recordOutput("Shooter/topVoltage", m_inputs.topVoltage);
    Logger.recordOutput("Shooter/bottomVoltage", m_inputs.bottomVoltage);
    Logger.recordOutput("Shooter/topVelocity", m_inputs.topVelocity);
    Logger.recordOutput("Shooter/bottomVelocity", m_inputs.bottomVelocity);
  }

  public void idlePeriodic() {
    m_io.setVoltage(ShooterConstants.kIdleVoltage, ShooterConstants.kIdleVoltage);
  }

  public void revvingPeriodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_topController.setPID(
              ShooterConstants.kTopShooterP.getAsDouble(),
              ShooterConstants.kTopShooterI.getAsDouble(),
              ShooterConstants.kTopShooterD.getAsDouble());
        },
        ShooterConstants.kTopShooterP,
        ShooterConstants.kTopShooterI,
        ShooterConstants.kTopShooterD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_bottomController.setPID(
              ShooterConstants.kBottomShooterP.getAsDouble(),
              ShooterConstants.kBottomShooterI.getAsDouble(),
              ShooterConstants.kBottomShooterD.getAsDouble());
        },
        ShooterConstants.kBottomShooterP,
        ShooterConstants.kBottomShooterI,
        ShooterConstants.kBottomShooterD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_topFF.setKs(ShooterConstants.kTopKs.getAsDouble());
          m_topFF.setKv(ShooterConstants.kTopKv.getAsDouble());
        },
        ShooterConstants.kTopKs,
        ShooterConstants.kTopKv);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_bottomFF.setKs(ShooterConstants.kBottomKs.getAsDouble());
          m_bottomFF.setKv(ShooterConstants.kTopKv.getAsDouble());
        },
        ShooterConstants.kBottomKs,
        ShooterConstants.kBottomKv);

    double topVoltage =
        m_topController.calculate(ShooterConstants.kTopRPS)
            + m_topFF.calculate(m_topController.getSetpoint());
    double bottomVoltage =
        m_bottomController.calculate(ShooterConstants.kBottomRPS)
            + m_bottomFF.calculate(m_bottomController.getSetpoint());

    m_io.setVoltage(topVoltage, bottomVoltage);
  }

  public void rejectingPeriodic() {
    m_io.setVoltage(ShooterConstants.kRejectingVoltage, ShooterConstants.kRejectingVoltage);
  }

  public void ampPeriodic() {
    setVelocity(ShooterConstants.kTopAmpVelocity, ShooterConstants.kBottomAmpVelocity);
    revvingPeriodic();
  }

  public void setVelocity(double top, double bottom) {
    m_topController.setSetpoint(top);
    m_bottomController.setSetpoint(bottom);
  }

  public void updateState(ShooterState state) {
    switch (state) {
      case kIdle:
      case kRejecting:
        setVelocity(0, 0);
      case kAmp:
      case kRevving:
        break;
    }
    m_profiles.setCurrentProfile(state);
  }
}
