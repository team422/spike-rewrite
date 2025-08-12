package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterInputs {
    double topVelocityRPS;
    double bottomVelocityRPS;
    double topVoltage;
    double bottomVoltage;
    double topCurrent;
    double bottomCurrent;
    boolean topMotorIsConnected;
    boolean bottomMotorIsConnected;
  }

  public void setVoltage(double topVoltage, double bottomVoltage);

  public void updateInputs(ShooterInputs inputs);
}
