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

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants constants;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Input signals
  private final StatusSignal<Angle> drivePositionRotations;
  private final StatusSignal<AngularVelocity> driveVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrentAmps;
  private final StatusSignal<Current> driveTorqueCurrentAmps;
  private final StatusSignal<Temperature> driveTemperatureCelcius;
  private double driveVelocityGoalRadiansPerSecond;
  private final StatusSignal<Double> driveVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> driveVelocityErrorRotationsPerSecond;

  private final StatusSignal<Angle> turnAbsolutePositionRotations;
  private final StatusSignal<Angle> turnPositionRotations;
  private final StatusSignal<AngularVelocity> turnVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrentAmps;
  private final StatusSignal<Current> turnTorqueCurrentAmps;
  private final StatusSignal<Temperature> turnTemperatureCelcius;
  private Rotation2d turnPositionGoal;
  private final StatusSignal<Double> turnPositionSetpointRotations;
  private final StatusSignal<Double> turnPositionErrorRotations;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Control requests
  private final VoltageOut voltageRequest;
  private final MotionMagicVoltage positionVoltageRequest;
  private final VelocityVoltage velocityVoltageRequest;

  private final TorqueCurrentFOC torqueCurrentRequest;
  private final MotionMagicTorqueCurrentFOC positionTorqueCurrentRequest;
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;

  public ModuleIOTalonFX(SwerveModuleConstants constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.DriveMotorId, DriveConstants.CAN_BUS);
    turnTalon = new TalonFX(constants.SteerMotorId, DriveConstants.CAN_BUS);
    cancoder = new CANcoder(constants.CANcoderId, DriveConstants.CAN_BUS);

    // Configure drive motor
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.CANcoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Create input signals
    drivePositionRotations = driveTalon.getPosition();
    driveVelocityRotationsPerSecond = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrentAmps = driveTalon.getStatorCurrent();
    driveTorqueCurrentAmps = driveTalon.getTorqueCurrent();
    driveTemperatureCelcius = driveTalon.getDeviceTemp();
    driveVelocityGoalRadiansPerSecond = 0.0;
    driveVelocitySetpointRotationsPerSecond = driveTalon.getClosedLoopReference();
    driveVelocityErrorRotationsPerSecond = driveTalon.getClosedLoopError();

    turnAbsolutePositionRotations = cancoder.getAbsolutePosition();
    turnPositionRotations = turnTalon.getPosition();
    turnVelocityRotationsPerSecond = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrentAmps = turnTalon.getStatorCurrent();
    turnTorqueCurrentAmps = turnTalon.getTorqueCurrent();
    turnTemperatureCelcius = turnTalon.getDeviceTemp();
    turnPositionGoal = new Rotation2d();
    turnPositionSetpointRotations = turnTalon.getClosedLoopReference();
    turnPositionErrorRotations = turnTalon.getClosedLoopError();

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());

    voltageRequest = new VoltageOut(0.0);
    positionVoltageRequest = new MotionMagicVoltage(0.0);
    velocityVoltageRequest = new VelocityVoltage(0.0);

    torqueCurrentRequest = new TorqueCurrentFOC(0.0);
    positionTorqueCurrentRequest = new MotionMagicTorqueCurrentFOC(0.0);
    velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_FREQUENCY, drivePositionRotations, turnPositionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocityRotationsPerSecond,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        driveTemperatureCelcius,
        driveVelocitySetpointRotationsPerSecond,
        driveVelocityErrorRotationsPerSecond,
        turnAbsolutePositionRotations,
        turnVelocityRotationsPerSecond,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTemperatureCelcius,
        turnPositionSetpointRotations,
        turnPositionErrorRotations);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        drivePositionRotations,
        driveVelocityRotationsPerSecond,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        turnTemperatureCelcius,
        driveVelocitySetpointRotationsPerSecond,
        driveVelocityErrorRotationsPerSecond);
    BaseStatusSignal.refreshAll(
        turnPositionRotations,
        turnVelocityRotationsPerSecond,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTemperatureCelcius,
        turnPositionSetpointRotations,
        turnPositionErrorRotations);
    BaseStatusSignal.refreshAll(turnAbsolutePositionRotations);

    // Update inputs
    inputs.drivePosition = Rotation2d.fromRotations(drivePositionRotations.getValueAsDouble());
    inputs.driveVelocityRadiansPerSecond =
        Units.rotationsToRadians(driveVelocityRotationsPerSecond.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrentAmps.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrentAmps.getValueAsDouble();
    inputs.driveTemperatureCelcius = driveTemperatureCelcius.getValueAsDouble();
    inputs.driveVelocityGoalRadiansPerSecond = driveVelocityGoalRadiansPerSecond;
    inputs.driveVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(driveVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.driveVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(driveVelocityErrorRotationsPerSecond.getValueAsDouble());

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePositionRotations.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPositionRotations.getValueAsDouble());
    inputs.turnVelocityRadiansPerSecond =
        Units.rotationsToRadians(turnVelocityRotationsPerSecond.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrentAmps.getValueAsDouble();
    inputs.turnTemperatureCelcius = turnTemperatureCelcius.getValueAsDouble();
    inputs.turnPositionGoal = turnPositionGoal;
    inputs.turnPositionSetpoint =
        Rotation2d.fromRotations(turnPositionSetpointRotations.getValueAsDouble());
    inputs.turnPositionError =
        Rotation2d.fromRotations(turnPositionErrorRotations.getValueAsDouble());

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositions =
        drivePositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadiansPerSecond) {
    driveVelocityGoalRadiansPerSecond = velocityRadiansPerSecond;
    double velocityRotPerSec = Units.radiansToRotations(velocityRadiansPerSecond);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnPositionGoal = rotation;
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }
}
