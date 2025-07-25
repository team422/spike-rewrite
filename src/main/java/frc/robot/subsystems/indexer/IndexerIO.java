package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  // TODO: make sim better
  @AutoLog
  public class IndexerInputs {
    double voltage;
    double velocityRPM;
    boolean hasPiece;
    boolean photoelectric1Raw;
    boolean photoelectric2Raw;
    double current;
    boolean isMotorConnected;
  }

  public void updateInputs(IndexerInputs inputs);

  public void setVoltage(double voltage);
}
