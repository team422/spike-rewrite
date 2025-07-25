package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIONeo implements IndexerIO {
  private SparkMax m_motor;
  private RelativeEncoder m_encoder;
  private DigitalInput m_sensor1;
  private DigitalInput m_sensor2;

  public IndexerIONeo(int motorPort, int sensorPort1, int sensorPort2) {
    m_motor = new SparkMax(motorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();

    m_sensor1 = new DigitalInput(sensorPort1);
    m_sensor2 = new DigitalInput(sensorPort2);

    SparkMaxConfig configs = new SparkMaxConfig();
    m_motor.configure(configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.voltage = m_motor.getBusVoltage() * m_motor.getAppliedOutput();
    inputs.velocityRPM = m_encoder.getVelocity();
    inputs.hasPiece = !m_sensor1.get() || !m_sensor2.get();
    inputs.current = m_motor.getOutputCurrent();
    inputs.photoelectric1Raw = m_sensor1.get();
    inputs.photoelectric2Raw = m_sensor2.get();
    inputs.isMotorConnected = m_motor.getBusVoltage() != 12;
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
