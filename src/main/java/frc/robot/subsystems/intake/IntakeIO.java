package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeInputs {
    double voltage;
    double velocityRPS;
    double current;
    boolean isMotorConnected;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double voltage);
}
