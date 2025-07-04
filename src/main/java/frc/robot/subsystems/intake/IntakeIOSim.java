package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim m_sim;
  private double m_voltage;

  public IntakeIOSim() {
    m_sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.kDCMotor, IntakeConstants.kSimMOI, IntakeConstants.kSimGearing),
            IntakeConstants.kDCMotor);
    m_voltage = 0.0;
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.velocity = m_sim.getAngularVelocityRadPerSec();
    inputs.voltage = m_voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }
}
