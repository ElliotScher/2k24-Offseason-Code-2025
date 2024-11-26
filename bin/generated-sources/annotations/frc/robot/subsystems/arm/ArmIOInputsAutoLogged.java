package frc.robot.subsystems.arm;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ArmPosition", armPosition);
    table.put("ArmVelocityRadPerSec", armVelocityRadPerSec);
    table.put("ArmAppliedVolts", armAppliedVolts);
    table.put("ArmCurrentAmps", armCurrentAmps);
    table.put("ArmTemperatureCelsius", armTemperatureCelsius);
    table.put("ArmAbsolutePosition", armAbsolutePosition);
    table.put("PositionSetpoint", positionSetpoint);
    table.put("PositionError", positionError);
    table.put("PositionGoal", positionGoal);
  }

  @Override
  public void fromLog(LogTable table) {
    armPosition = table.get("ArmPosition", armPosition);
    armVelocityRadPerSec = table.get("ArmVelocityRadPerSec", armVelocityRadPerSec);
    armAppliedVolts = table.get("ArmAppliedVolts", armAppliedVolts);
    armCurrentAmps = table.get("ArmCurrentAmps", armCurrentAmps);
    armTemperatureCelsius = table.get("ArmTemperatureCelsius", armTemperatureCelsius);
    armAbsolutePosition = table.get("ArmAbsolutePosition", armAbsolutePosition);
    positionSetpoint = table.get("PositionSetpoint", positionSetpoint);
    positionError = table.get("PositionError", positionError);
    positionGoal = table.get("PositionGoal", positionGoal);
  }

  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();
    copy.armPosition = this.armPosition;
    copy.armVelocityRadPerSec = this.armVelocityRadPerSec;
    copy.armAppliedVolts = this.armAppliedVolts;
    copy.armCurrentAmps = this.armCurrentAmps;
    copy.armTemperatureCelsius = this.armTemperatureCelsius;
    copy.armAbsolutePosition = this.armAbsolutePosition;
    copy.positionSetpoint = this.positionSetpoint;
    copy.positionError = this.positionError;
    copy.positionGoal = this.positionGoal;
    return copy;
  }
}
