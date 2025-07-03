// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {

  private final SparkMax m_driveSparkMax;
  private final SparkMax m_turnSparkMax;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final CANcoder m_cancoder;
  private final Queue<Double> m_timestampQueue;
  private final Queue<Double> m_drivePositionQueue;
  private final Queue<Double> m_turnPositionQueue;

  private StatusSignal<Angle> m_turnAbsolutePosition;

  private final boolean m_isTurnMotorInverted = true;
  private final Rotation2d m_absoluteEncoderOffset;

  private final SparkBaseConfig m_driveConfig;
  private final SparkBaseConfig m_turnConfig;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        m_driveSparkMax = new SparkMax(Ports.kFrontLeftDrive, MotorType.kBrushless);
        m_turnSparkMax = new SparkMax(Ports.kFrontLeftTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kFrontLeftCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        m_driveSparkMax = new SparkMax(Ports.kFrontRightDrive, MotorType.kBrushless);
        m_turnSparkMax = new SparkMax(Ports.kFrontRightTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kFrontRightCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        m_driveSparkMax = new SparkMax(Ports.kBackLeftDrive, MotorType.kBrushless);
        m_turnSparkMax = new SparkMax(Ports.kBackLeftTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kBackLeftCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        m_driveSparkMax = new SparkMax(Ports.kBackRightDrive, MotorType.kBrushless);
        m_turnSparkMax = new SparkMax(Ports.kBackRightTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kBackRightCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var encoderConfig = new EncoderConfig().uvwAverageDepth(2).uvwMeasurementPeriod(10);
    var signalConfig =
        new SignalsConfig()
            .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.kOdometryFrequency));
    var driveLoopConfig =
        new ClosedLoopConfig()
            .pidf(
                DriveConstants.kDriveToPointP.getAsDouble(),
                DriveConstants.kDriveToPointI.getAsDouble(),
                DriveConstants.kDriveToPointD.getAsDouble(),
                DriveConstants.kDriveFF.getAsDouble());
    m_driveConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(40)
            .voltageCompensation(12)
            .apply(encoderConfig)
            .apply(signalConfig)
            .apply(driveLoopConfig);

    var turnLoopConfig =
        new ClosedLoopConfig()
            .pidf(
                DriveConstants.kDriveToPointHeadingP.getAsDouble(),
                DriveConstants.kDriveToPointHeadingI.getAsDouble(),
                DriveConstants.kDriveToPointHeadingD.getAsDouble(),
                DriveConstants.kTurnFF.getAsDouble());
    m_turnConfig =
        new SparkMaxConfig()
            .inverted(m_isTurnMotorInverted)
            .smartCurrentLimit(30)
            .voltageCompensation(12)
            .apply(encoderConfig)
            .apply(signalConfig)
            .apply(turnLoopConfig);

    m_driveSparkMax.setCANTimeout(250);
    m_turnSparkMax.setCANTimeout(250);

    m_driveEncoder = m_driveSparkMax.getEncoder();
    m_turnRelativeEncoder = m_turnSparkMax.getEncoder();

    m_driveEncoder.setPosition(0.0);
    m_turnRelativeEncoder.setPosition(0.0);

    m_driveSparkMax.setCANTimeout(0);
    m_turnSparkMax.setCANTimeout(0);

    m_timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    m_drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = m_driveEncoder.getPosition();
                  if (m_driveSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    m_turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = m_turnRelativeEncoder.getPosition();
                  if (m_turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

    m_driveSparkMax.configure(
        m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnSparkMax.configure(
        m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CANcoderConfiguration config = new CANcoderConfiguration();
    m_cancoder.getConfigurator().refresh(config.MagnetSensor);
    m_cancoder.getConfigurator().apply(config);

    m_turnAbsolutePosition = m_cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(75.0, m_turnAbsolutePosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(m_turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(m_driveEncoder.getPosition()) / DriveConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity())
            / DriveConstants.kDriveGearRatio;
    inputs.driveAppliedVolts = m_driveSparkMax.getAppliedOutput() * m_driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = m_driveSparkMax.getOutputCurrent();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble())
            .minus(m_absoluteEncoderOffset);

    inputs.turnPosition =
        Rotation2d.fromRotations(
            m_turnRelativeEncoder.getPosition() / DriveConstants.kTurnGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity())
            / DriveConstants.kTurnGearRatio;
    inputs.turnAppliedVolts = m_turnSparkMax.getAppliedOutput() * m_turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = m_turnSparkMax.getOutputCurrent();

    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream()
            .mapToDouble(
                (Double value) -> Units.rotationsToRadians(value) / DriveConstants.kDriveGearRatio)
            .toArray();
    inputs.odometryTurnPositions =
        m_turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / DriveConstants.kTurnGearRatio))
            .toArray(Rotation2d[]::new);
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    var configs = new ClosedLoopConfig().velocityFF(feedforward);
    m_driveConfig.apply(configs);

    // todo: how to do velocity
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTurnPosition'");
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    m_driveConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    m_turnConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_driveConfig.smartCurrentLimit((int) supplyLimit);
    m_turnConfig.smartCurrentLimit((int) supplyLimit);
  }

  @Override
  public void setDriveRawOutput(double output) {
    m_driveSparkMax.setVoltage(output);
  }

  @Override
  public void setTurnRawOutput(double output) {
    m_turnSparkMax.setVoltage(output);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    var configs = new ClosedLoopConfig().pid(kP, kI, kD);
    m_driveConfig.apply(configs);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    var configs = new ClosedLoopConfig().pid(kP, kI, kD);
    m_turnConfig.apply(configs);
  }

  @Override
  public void resetTurnMotor(Angle position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetTurnMotor'");
  }
}
