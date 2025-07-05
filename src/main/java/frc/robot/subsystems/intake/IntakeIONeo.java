package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeIONeo implements IntakeIO {
  private SparkMax m_motor;
  private RelativeEncoder m_encoder;

  public IntakeIONeo(int motorPort) {
    m_motor = new SparkMax(motorPort, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
  }

  public void updateInputs(IntakeInputs inputs) {
    inputs.velocity = m_encoder.getVelocity();
    inputs.voltage = m_motor.getBusVoltage() * m_motor.getAppliedOutput();
  }

  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
