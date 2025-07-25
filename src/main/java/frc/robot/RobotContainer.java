// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOReplay;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.indexer.IndexerIONeo;
import frc.robot.subsystems.indexer.IndexerIOReplay;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIONeo;
import frc.robot.subsystems.intake.IntakeIOReplay;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIONeo;
import frc.robot.subsystems.shooter.ShooterIOReplay;
import frc.robot.subsystems.shooter.ShooterIOSim;

public class RobotContainer {
  private RobotState m_state;

  private Drive m_drive;
  private Indexer m_indexer;
  private Intake m_intake;
  private Shooter m_shooter;

  private DriverControls m_controls;

  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureController();
    configureBindings();
  }

  private void configureSubsystems() {
    switch (Constants.kCurrentMode) {
      case REAL:
        m_drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        m_indexer =
            new Indexer(
                new IndexerIONeo(Ports.kIndexer, Ports.kPhotoelectric1, Ports.kPhotoelectric2));
        m_intake = new Intake(new IntakeIONeo(Ports.kIntake));
        m_shooter =
            new Shooter(
                new ShooterIONeo(Ports.kTopShooter, Ports.kBottomShooter),
                new PIDController(
                    ShooterConstants.kTopShooterP.getAsDouble(),
                    ShooterConstants.kTopShooterI.getAsDouble(),
                    ShooterConstants.kTopShooterD.getAsDouble()),
                new PIDController(
                    ShooterConstants.kBottomShooterP.getAsDouble(),
                    ShooterConstants.kBottomShooterI.getAsDouble(),
                    ShooterConstants.kBottomShooterD.getAsDouble()),
                new SimpleMotorFeedforward(
                    ShooterConstants.kTopKs.getAsDouble(), ShooterConstants.kTopKv.getAsDouble()),
                new SimpleMotorFeedforward(
                    ShooterConstants.kBottomKs.getAsDouble(),
                    ShooterConstants.kBottomKv.getAsDouble()));
        break;
      case SIM:
        m_drive =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_indexer = new Indexer(new IndexerIOSim());
        m_intake = new Intake(new IntakeIOSim());
        m_shooter =
            new Shooter(
                new ShooterIOSim(),
                new PIDController(
                    ShooterConstants.kTopShooterP.getAsDouble(),
                    ShooterConstants.kTopShooterI.getAsDouble(),
                    ShooterConstants.kTopShooterD.getAsDouble()),
                new PIDController(
                    ShooterConstants.kBottomShooterP.getAsDouble(),
                    ShooterConstants.kBottomShooterI.getAsDouble(),
                    ShooterConstants.kBottomShooterD.getAsDouble()),
                new SimpleMotorFeedforward(
                    ShooterConstants.kTopKs.getAsDouble(), ShooterConstants.kTopKv.getAsDouble()),
                new SimpleMotorFeedforward(
                    ShooterConstants.kBottomKs.getAsDouble(),
                    ShooterConstants.kBottomKv.getAsDouble()));
        break;
      case REPLAY:
        m_drive =
            new Drive(
                new GyroIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay());
        m_indexer = new Indexer(new IndexerIOReplay());
        m_intake = new Intake(new IntakeIOReplay());
        m_shooter =
            new Shooter(
                new ShooterIOReplay(),
                new PIDController(
                    ShooterConstants.kTopShooterP.getAsDouble(),
                    ShooterConstants.kTopShooterI.getAsDouble(),
                    ShooterConstants.kTopShooterD.getAsDouble()),
                new PIDController(
                    ShooterConstants.kBottomShooterP.getAsDouble(),
                    ShooterConstants.kBottomShooterI.getAsDouble(),
                    ShooterConstants.kBottomShooterD.getAsDouble()),
                new SimpleMotorFeedforward(
                    ShooterConstants.kTopKs.getAsDouble(), ShooterConstants.kTopKv.getAsDouble()),
                new SimpleMotorFeedforward(
                    ShooterConstants.kBottomKs.getAsDouble(),
                    ShooterConstants.kBottomKv.getAsDouble()));
        break;
    }

    RobotState.startInstance(m_drive, m_shooter, m_indexer, m_intake);
    m_state = RobotState.getInstance();
  }

  private void configureCommands() {}

  private void configureController() {
    m_controls = new DriverControlsPS5(0);
    // m_controls = new DriverControlsXbox(0);
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive, m_controls::getForward, m_controls::getStrafe, m_controls::getTurn, false));
    m_controls
        .intake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_state.updateAction(RobotAction.kIntake);
                }));
    m_controls
        .rev()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_state.updateAction(RobotAction.kSubwooferShooting);
                }));
    m_controls
        .index()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_indexer.updateState(IndexerState.kShooting);
                }));
    m_controls
        .eject()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_state.updateAction(RobotAction.kVomitting);
                }));
    m_controls
        .amp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_state.updateAction(RobotAction.kAmp);
                }));
    m_controls
        .align()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_state.updateAction(RobotAction.kAlignShooting);
                }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
