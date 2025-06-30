package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterInputs {
    double topVelocity;
    double bottomVelocity;
    double topVoltage;
    double bottomVoltage;
    boolean withinTolerance;
  }

  public void setVoltage(double topVoltage, double bottomVoltage);

  public void updateInputs(ShooterInputs inputs);
}
