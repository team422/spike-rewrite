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
  private RelativeEncoder m_bottomCoder;
  private SparkMaxConfig m_bottomConfigs;

  public ShooterIONeo(int topPort, int bottomPort) {
    m_topWheel = new SparkMax(topPort, MotorType.kBrushless);
    m_bottomWheel = new SparkMax(bottomPort, MotorType.kBrushless);

    m_topCoder = m_topWheel.getEncoder();
    m_bottomCoder = m_bottomWheel.getEncoder();

    m_bottomConfigs = new SparkMaxConfig();
    m_bottomConfigs.inverted(true);

    m_bottomWheel.configure(
        m_bottomConfigs,
        ResetMode.kResetSafeParameters,
        PersistMode
            .kPersistParameters); // documentation for ResetMode and PersistMode is just awful, I
    // can't tell what it means
  }

  @Override
  public void setVoltage(double topVoltage, double bottomVoltage) {
    m_topWheel.setVoltage(topVoltage);
    m_bottomWheel.setVoltage(bottomVoltage);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.topVelocity = m_topCoder.getVelocity();
    inputs.bottomVelocity = m_bottomCoder.getVelocity();
    inputs.topVoltage = m_topWheel.getBusVoltage() * m_topWheel.getAppliedOutput();
    inputs.bottomVoltage = m_bottomWheel.getBusVoltage() * m_bottomWheel.getAppliedOutput();
  }
}
