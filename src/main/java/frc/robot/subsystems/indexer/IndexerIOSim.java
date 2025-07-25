package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.Ports;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim m_sim;
  private double m_voltage;
  private DigitalInput m_sensor1;
  private DigitalInput m_sensor2;

  public IndexerIOSim() {
    m_sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IndexerConstants.kDCMotor, IndexerConstants.kSimMOI, IndexerConstants.kSimGearing),
            IndexerConstants.kDCMotor);
    m_voltage = 0.0;
    m_sensor1 = new DigitalInput(Ports.kPhotoelectric1);
    m_sensor2 = new DigitalInput(Ports.kPhotoelectric2);
  }

  public void updateInputs(IndexerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.hasPiece = !m_sensor1.get() || !m_sensor2.get();
    inputs.voltage = m_voltage;
    inputs.velocityRPM = m_sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }
}
