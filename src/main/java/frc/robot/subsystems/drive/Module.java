package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private boolean m_turnRelativeReset = false; // whether we reset the turn motor already
  private SwerveModulePosition[] m_odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.m_io = io;
    this.m_index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    if (RobotBase.isReal()) {
      m_driveFeedforward = new SimpleMotorFeedforward(0.22810, 0.13319);
      m_io.setDrivePID(2.0, 0.0, 0.0);
      m_io.setTurnPID(300.0, 0.0, 0.0);
    } else {
      m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
      m_io.setDrivePID(0.1, 0.0, 0.0);
      m_io.setTurnPID(10.0, 0.0, 0.0);
    }

    setBrakeMode(true);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(m_index), m_inputs);
  }

  public void periodic() {
    if (!m_turnRelativeReset && m_inputs.turnEncoderIsConnected) {
      m_io.resetTurnMotor(m_inputs.turnAbsolutePosition.getMeasure());
      m_turnRelativeReset = true;
    }

    // Calculate positions for odometry
    int sampleCount = m_inputs.odometryTimestamps.length; // All signals are sampled together
    m_odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = m_inputs.odometryDrivePositionsRad[i] * DriveConstants.kWheelRadius;
      Rotation2d angle = m_inputs.odometryTurnPositions[i];
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. */
  public void runSetpoint(SwerveModuleState state) {
    state.optimize(getAngle());

    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.kWheelRadius;

    m_io.setDriveVelocity(speedRadPerSec, m_driveFeedforward.calculate(speedRadPerSec));
    m_io.setTurnPosition(state.angle);

    Logger.recordOutput(
        "Module" + m_index + "/DriveFF", m_driveFeedforward.calculate(speedRadPerSec));
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    // Closed loop turn control
    m_io.setTurnPosition(new Rotation2d());

    // Open loop drive control
    m_io.setDriveRawOutput(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    m_io.setDriveRawOutput(0.0);
    m_io.setTurnRawOutput(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(enabled);
    m_io.setTurnBrakeMode(enabled);
  }

  /** Sets whether brake mode is enabled, only for the drive motor. */
  public void setDriveBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return m_inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * DriveConstants.kWheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * DriveConstants.kWheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return m_odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return m_inputs.odometryTimestamps;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getDriveVelocity() {
    return m_inputs.driveVelocityRadPerSec;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return m_inputs.drivePositionRad;
  }

  /**
   * Sets the current limit for the drive motor
   *
   * @param supplyLimit The current limit in amps
   */
  public void setCurrentLimits(double supplyLimit) {
    m_io.setCurrentLimits(supplyLimit);
  }

  /** Returns the drive motor's acceleration in radians/sec^2. */
  public double getDriveAcceleration() {
    return m_inputs.driveAccelerationRadPerSecSq;
  }

  /** Returns the drive motor's supply current in amps. */
  public double getDriveCurrent() {
    return m_inputs.driveCurrentAmps;
  }
}
