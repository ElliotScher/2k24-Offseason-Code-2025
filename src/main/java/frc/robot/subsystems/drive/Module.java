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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private final LoggedTunableNumber driveKp;
  private final LoggedTunableNumber driveKd;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);

    driveKp = new LoggedTunableNumber("Drive/Drive Kp", DriveConstants.GAINS.driveKp());
    driveKd = new LoggedTunableNumber("Drive/Drive Kd", DriveConstants.GAINS.driveKd());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositions[i].getRadians() * DriveConstants.DRIVE_CONFIG.wheelRadius();
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setDrivePID(driveKp.get(), 0.0, driveKd.get()), driveKp, driveKd);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state, SwerveModuleState torqueFeedforward) {
    // Optimize veloci[ty setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    double wheelTorqueNewtonMeters = torqueFeedforward.speedMetersPerSecond;
    // Apply setpoints
    io.setDriveVelocity(
        state.speedMetersPerSecond / DriveConstants.DRIVE_CONFIG.wheelRadius(),
        DriveConstants.DRIVE_CONFIG
            .driveModel()
            .getCurrent(wheelTorqueNewtonMeters / DriveConstants.FRONT_LEFT.DriveMotorGearRatio));
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double amps) {
    io.setDriveAmps(amps);
    io.setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveAmps(0.0);
    io.setTurnAmps(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePosition.getRadians() * DriveConstants.DRIVE_CONFIG.wheelRadius();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadiansPerSecond * DriveConstants.DRIVE_CONFIG.wheelRadius();
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
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public Rotation2d getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosition;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadiansPerSecond);
  }
}
