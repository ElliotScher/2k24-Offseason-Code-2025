package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePosition", drivePosition);
    table.put("DriveVelocityRadiansPerSecond", driveVelocityRadiansPerSecond);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    table.put("DriveTorqueCurrentAmps", driveTorqueCurrentAmps);
    table.put("DriveTemperatureCelcius", driveTemperatureCelcius);
    table.put("DriveVelocitySetpointRadiansPerSecond", driveVelocitySetpointRadiansPerSecond);
    table.put("DriveVelocityErrorRadiansPerSecond", driveVelocityErrorRadiansPerSecond);
    table.put("TurnAbsolutePosition", turnAbsolutePosition);
    table.put("TurnPosition", turnPosition);
    table.put("TurnVelocityRadiansPerSecond", turnVelocityRadiansPerSecond);
    table.put("TurnAppliedVolts", turnAppliedVolts);
    table.put("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    table.put("TurnTorqueCurrentAmps", turnTorqueCurrentAmps);
    table.put("TurnTemperatureCelcius", turnTemperatureCelcius);
    table.put("TurnPositionGoal", turnPositionGoal);
    table.put("TurnPositionSetpoint", turnPositionSetpoint);
    table.put("TurnPositionError", turnPositionError);
    table.put("DriveConnected", driveConnected);
    table.put("TurnConnected", turnConnected);
    table.put("TurnEncoderConnected", turnEncoderConnected);
    table.put("OdometryTimestamps", odometryTimestamps);
    table.put("OdometryDrivePositions", odometryDrivePositions);
    table.put("OdometryTurnPositions", odometryTurnPositions);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePosition = table.get("DrivePosition", drivePosition);
    driveVelocityRadiansPerSecond =
        table.get("DriveVelocityRadiansPerSecond", driveVelocityRadiansPerSecond);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveSupplyCurrentAmps = table.get("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    driveTorqueCurrentAmps = table.get("DriveTorqueCurrentAmps", driveTorqueCurrentAmps);
    driveTemperatureCelcius = table.get("DriveTemperatureCelcius", driveTemperatureCelcius);
    driveVelocitySetpointRadiansPerSecond =
        table.get("DriveVelocitySetpointRadiansPerSecond", driveVelocitySetpointRadiansPerSecond);
    driveVelocityErrorRadiansPerSecond =
        table.get("DriveVelocityErrorRadiansPerSecond", driveVelocityErrorRadiansPerSecond);
    turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
    turnPosition = table.get("TurnPosition", turnPosition);
    turnVelocityRadiansPerSecond =
        table.get("TurnVelocityRadiansPerSecond", turnVelocityRadiansPerSecond);
    turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
    turnSupplyCurrentAmps = table.get("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    turnTorqueCurrentAmps = table.get("TurnTorqueCurrentAmps", turnTorqueCurrentAmps);
    turnTemperatureCelcius = table.get("TurnTemperatureCelcius", turnTemperatureCelcius);
    turnPositionGoal = table.get("TurnPositionGoal", turnPositionGoal);
    turnPositionSetpoint = table.get("TurnPositionSetpoint", turnPositionSetpoint);
    turnPositionError = table.get("TurnPositionError", turnPositionError);
    driveConnected = table.get("DriveConnected", driveConnected);
    turnConnected = table.get("TurnConnected", turnConnected);
    turnEncoderConnected = table.get("TurnEncoderConnected", turnEncoderConnected);
    odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
    odometryDrivePositions = table.get("OdometryDrivePositions", odometryDrivePositions);
    odometryTurnPositions = table.get("OdometryTurnPositions", odometryTurnPositions);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.drivePosition = this.drivePosition;
    copy.driveVelocityRadiansPerSecond = this.driveVelocityRadiansPerSecond;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveSupplyCurrentAmps = this.driveSupplyCurrentAmps;
    copy.driveTorqueCurrentAmps = this.driveTorqueCurrentAmps;
    copy.driveTemperatureCelcius = this.driveTemperatureCelcius;
    copy.driveVelocitySetpointRadiansPerSecond = this.driveVelocitySetpointRadiansPerSecond;
    copy.driveVelocityErrorRadiansPerSecond = this.driveVelocityErrorRadiansPerSecond;
    copy.turnAbsolutePosition = this.turnAbsolutePosition;
    copy.turnPosition = this.turnPosition;
    copy.turnVelocityRadiansPerSecond = this.turnVelocityRadiansPerSecond;
    copy.turnAppliedVolts = this.turnAppliedVolts;
    copy.turnSupplyCurrentAmps = this.turnSupplyCurrentAmps;
    copy.turnTorqueCurrentAmps = this.turnTorqueCurrentAmps;
    copy.turnTemperatureCelcius = this.turnTemperatureCelcius;
    copy.turnPositionGoal = this.turnPositionGoal;
    copy.turnPositionSetpoint = this.turnPositionSetpoint;
    copy.turnPositionError = this.turnPositionError;
    copy.driveConnected = this.driveConnected;
    copy.turnConnected = this.turnConnected;
    copy.turnEncoderConnected = this.turnEncoderConnected;
    copy.odometryTimestamps = this.odometryTimestamps.clone();
    copy.odometryDrivePositions = this.odometryDrivePositions.clone();
    copy.odometryTurnPositions = this.odometryTurnPositions.clone();
    return copy;
  }
}
