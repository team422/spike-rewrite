package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim m_topSim;
  private FlywheelSim m_bottomSim;
  private double m_topVoltage;
  private double m_bottomVoltage;

  public ShooterIOSim() {
    m_topSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                ShooterConstants.kTopDCMotor,
                ShooterConstants.kSimMOI,
                ShooterConstants.kSimGearing),
            ShooterConstants.kTopDCMotor);
    m_bottomSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                ShooterConstants.kBottomDCMotor,
                ShooterConstants.kSimMOI,
                ShooterConstants.kSimGearing),
            ShooterConstants.kBottomDCMotor);
    m_topVoltage = 0.0;
    m_bottomVoltage = 0.0;
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    m_topSim.setInputVoltage(m_topVoltage);
    m_bottomSim.setInputVoltage(m_bottomVoltage);
    m_topSim.update(.02);
    m_bottomSim.update(.02);

    inputs.topVoltage = m_topVoltage;
    inputs.bottomVoltage = m_bottomVoltage;
    inputs.topVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(m_topSim.getAngularVelocityRadPerSec());
    inputs.bottomVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(m_bottomSim.getAngularVelocityRadPerSec());
  }

  @Override
  public void setVoltage(double topVoltage, double bottomVoltage) {
    m_topVoltage = topVoltage;
    m_bottomVoltage = bottomVoltage;
  }
}
