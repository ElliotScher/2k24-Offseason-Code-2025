package frc.robot.subsystems.shooter;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TopPosition", topPosition);
    table.put("TopVelocityRadPerSec", topVelocityRadPerSec);
    table.put("TopAppliedVolts", topAppliedVolts);
    table.put("TopCurrentAmps", topCurrentAmps);
    table.put("TopTemperatureCelsius", topTemperatureCelsius);
    table.put("BottomPosition", bottomPosition);
    table.put("BottomVelocityRadPerSec", bottomVelocityRadPerSec);
    table.put("BottomAppliedVolts", bottomAppliedVolts);
    table.put("BottomCurrentAmps", bottomCurrentAmps);
    table.put("BottomTemperatureCelsius", bottomTemperatureCelsius);
    table.put("TopVelocityGoalRadiansPerSec", topVelocityGoalRadiansPerSec);
    table.put("BottomVelocityGoalRadiansPerSec", bottomVelocityGoalRadiansPerSec);
    table.put("TopVelocitySetpointRadiansPerSec", topVelocitySetpointRadiansPerSec);
    table.put("BottomVelocitySetpointRadiansPerSec", bottomVelocitySetpointRadiansPerSec);
    table.put("TopVelocityErrorRadiansPerSec", topVelocityErrorRadiansPerSec);
    table.put("BottomVelocityErrorRadiansPerSec", bottomVelocityErrorRadiansPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    topPosition = table.get("TopPosition", topPosition);
    topVelocityRadPerSec = table.get("TopVelocityRadPerSec", topVelocityRadPerSec);
    topAppliedVolts = table.get("TopAppliedVolts", topAppliedVolts);
    topCurrentAmps = table.get("TopCurrentAmps", topCurrentAmps);
    topTemperatureCelsius = table.get("TopTemperatureCelsius", topTemperatureCelsius);
    bottomPosition = table.get("BottomPosition", bottomPosition);
    bottomVelocityRadPerSec = table.get("BottomVelocityRadPerSec", bottomVelocityRadPerSec);
    bottomAppliedVolts = table.get("BottomAppliedVolts", bottomAppliedVolts);
    bottomCurrentAmps = table.get("BottomCurrentAmps", bottomCurrentAmps);
    bottomTemperatureCelsius = table.get("BottomTemperatureCelsius", bottomTemperatureCelsius);
    topVelocityGoalRadiansPerSec = table.get("TopVelocityGoalRadiansPerSec", topVelocityGoalRadiansPerSec);
    bottomVelocityGoalRadiansPerSec = table.get("BottomVelocityGoalRadiansPerSec", bottomVelocityGoalRadiansPerSec);
    topVelocitySetpointRadiansPerSec = table.get("TopVelocitySetpointRadiansPerSec", topVelocitySetpointRadiansPerSec);
    bottomVelocitySetpointRadiansPerSec = table.get("BottomVelocitySetpointRadiansPerSec", bottomVelocitySetpointRadiansPerSec);
    topVelocityErrorRadiansPerSec = table.get("TopVelocityErrorRadiansPerSec", topVelocityErrorRadiansPerSec);
    bottomVelocityErrorRadiansPerSec = table.get("BottomVelocityErrorRadiansPerSec", bottomVelocityErrorRadiansPerSec);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.topPosition = this.topPosition;
    copy.topVelocityRadPerSec = this.topVelocityRadPerSec;
    copy.topAppliedVolts = this.topAppliedVolts;
    copy.topCurrentAmps = this.topCurrentAmps;
    copy.topTemperatureCelsius = this.topTemperatureCelsius;
    copy.bottomPosition = this.bottomPosition;
    copy.bottomVelocityRadPerSec = this.bottomVelocityRadPerSec;
    copy.bottomAppliedVolts = this.bottomAppliedVolts;
    copy.bottomCurrentAmps = this.bottomCurrentAmps;
    copy.bottomTemperatureCelsius = this.bottomTemperatureCelsius;
    copy.topVelocityGoalRadiansPerSec = this.topVelocityGoalRadiansPerSec;
    copy.bottomVelocityGoalRadiansPerSec = this.bottomVelocityGoalRadiansPerSec;
    copy.topVelocitySetpointRadiansPerSec = this.topVelocitySetpointRadiansPerSec;
    copy.bottomVelocitySetpointRadiansPerSec = this.bottomVelocitySetpointRadiansPerSec;
    copy.topVelocityErrorRadiansPerSec = this.topVelocityErrorRadiansPerSec;
    copy.bottomVelocityErrorRadiansPerSec = this.bottomVelocityErrorRadiansPerSec;
    return copy;
  }
}
