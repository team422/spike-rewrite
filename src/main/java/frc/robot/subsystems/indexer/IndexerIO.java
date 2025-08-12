package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    double voltage;
    double velocityRPS;
    boolean hasPiece;
    boolean photoelectric1Raw;
    boolean photoelectric2Raw;
    double current;
    boolean isMotorConnected;
  }

  public void updateInputs(IndexerInputs inputs);

  public void setVoltage(double voltage);
}
