package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIONeo implements IntakeIO {
  private SparkMax m_motor;
  private RelativeEncoder m_encoder;

  public IntakeIONeo(int motorPort) {
    m_motor = new SparkMax(motorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();

    SparkMaxConfig configs = new SparkMaxConfig();
    m_motor.configure(configs, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IntakeInputs inputs) {
    inputs.velocityRPS = m_encoder.getVelocity() / 60;
    inputs.voltage = m_motor.getBusVoltage() * m_motor.getAppliedOutput();
    inputs.current = m_motor.getOutputCurrent();
    inputs.isMotorConnected = m_motor.getBusVoltage() != 12;
  }

  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
