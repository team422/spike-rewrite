package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    double voltage;
    double velocity;
    boolean hasPiece;
  }

  public void updateInputs(IndexerInputs inputs);

  public void setVoltage(double voltage);

  public boolean hasPiece();
}
