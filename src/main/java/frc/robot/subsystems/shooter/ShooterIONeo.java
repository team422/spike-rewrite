package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIONeo implements ShooterIO {
  private SparkMax m_topWheel;
  private SparkMax m_bottomWheel;
  private RelativeEncoder m_topCoder;
  private SparkMaxConfig m_topConfigs;
  private RelativeEncoder m_bottomCoder;
  private SparkMaxConfig m_bottomConfigs;

  public ShooterIONeo(int topPort, int bottomPort) {
    m_topWheel = new SparkMax(topPort, MotorType.kBrushless);
    m_bottomWheel = new SparkMax(bottomPort, MotorType.kBrushless);

    m_topCoder = m_topWheel.getEncoder();
    m_bottomCoder = m_bottomWheel.getEncoder();

    m_topConfigs = new SparkMaxConfig();

    m_bottomConfigs = new SparkMaxConfig();
    m_bottomConfigs.inverted(true);

    m_topWheel.configure(
        m_topConfigs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomWheel.configure(
        m_bottomConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double topVoltage, double bottomVoltage) {
    m_topWheel.setVoltage(topVoltage);
    m_bottomWheel.setVoltage(bottomVoltage);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.topVelocityRPM = m_topCoder.getVelocity() / 60;
    inputs.bottomVelocityRPM = m_bottomCoder.getVelocity() / 60;
    inputs.topVoltage = m_topWheel.getBusVoltage() * m_topWheel.getAppliedOutput();
    inputs.bottomVoltage = m_bottomWheel.getBusVoltage() * m_bottomWheel.getAppliedOutput();
    inputs.topCurrent = m_topWheel.getOutputCurrent();
    inputs.bottomCurrent = m_bottomWheel.getOutputCurrent();
    inputs.topMotorIsConnected = m_topWheel.getBusVoltage() != 12;
    inputs.bottomMotorIsConnected = m_bottomWheel.getBusVoltage() != 12;
  }
}
