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
    table.put("TurnAbsolutePosition", turnAbsolutePosition);
    table.put("TurnPosition", turnPosition);
    table.put("TurnVelocityRadiansPerSecond", turnVelocityRadiansPerSecond);
    table.put("TurnAppliedVolts", turnAppliedVolts);
    table.put("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    table.put("TurnTorqueCurrentAmps", turnTorqueCurrentAmps);
    table.put("TurnTemperatureCelcius", turnTemperatureCelcius);
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
    turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
    turnPosition = table.get("TurnPosition", turnPosition);
    turnVelocityRadiansPerSecond =
        table.get("TurnVelocityRadiansPerSecond", turnVelocityRadiansPerSecond);
    turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
    turnSupplyCurrentAmps = table.get("TurnSupplyCurrentAmps", turnSupplyCurrentAmps);
    turnTorqueCurrentAmps = table.get("TurnTorqueCurrentAmps", turnTorqueCurrentAmps);
    turnTemperatureCelcius = table.get("TurnTemperatureCelcius", turnTemperatureCelcius);
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
    copy.turnAbsolutePosition = this.turnAbsolutePosition;
    copy.turnPosition = this.turnPosition;
    copy.turnVelocityRadiansPerSecond = this.turnVelocityRadiansPerSecond;
    copy.turnAppliedVolts = this.turnAppliedVolts;
    copy.turnSupplyCurrentAmps = this.turnSupplyCurrentAmps;
    copy.turnTorqueCurrentAmps = this.turnTorqueCurrentAmps;
    copy.turnTemperatureCelcius = this.turnTemperatureCelcius;
    copy.driveConnected = this.driveConnected;
    copy.turnConnected = this.turnConnected;
    copy.turnEncoderConnected = this.turnEncoderConnected;
    copy.odometryTimestamps = this.odometryTimestamps.clone();
    copy.odometryDrivePositions = this.odometryDrivePositions.clone();
    copy.odometryTurnPositions = this.odometryTurnPositions.clone();
    return copy;
  }
}
