package frc.robot.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
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
    table.put("AcceleratorPosition", acceleratorPosition);
    table.put("AcceleratorVelocityRadPerSec", acceleratorVelocityRadPerSec);
    table.put("AcceleratorAppliedVolts", acceleratorAppliedVolts);
    table.put("AcceleratorCurrentAmps", acceleratorCurrentAmps);
    table.put("AcceleratorTemperatureCelsius", acceleratorTemperatureCelsius);
    table.put("IntakeSensor", intakeSensor);
    table.put("MiddleSensor", middleSensor);
    table.put("FinalSensor", finalSensor);
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
    acceleratorPosition = table.get("AcceleratorPosition", acceleratorPosition);
    acceleratorVelocityRadPerSec = table.get("AcceleratorVelocityRadPerSec", acceleratorVelocityRadPerSec);
    acceleratorAppliedVolts = table.get("AcceleratorAppliedVolts", acceleratorAppliedVolts);
    acceleratorCurrentAmps = table.get("AcceleratorCurrentAmps", acceleratorCurrentAmps);
    acceleratorTemperatureCelsius = table.get("AcceleratorTemperatureCelsius", acceleratorTemperatureCelsius);
    intakeSensor = table.get("IntakeSensor", intakeSensor);
    middleSensor = table.get("MiddleSensor", middleSensor);
    finalSensor = table.get("FinalSensor", finalSensor);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
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
    copy.acceleratorPosition = this.acceleratorPosition;
    copy.acceleratorVelocityRadPerSec = this.acceleratorVelocityRadPerSec;
    copy.acceleratorAppliedVolts = this.acceleratorAppliedVolts;
    copy.acceleratorCurrentAmps = this.acceleratorCurrentAmps;
    copy.acceleratorTemperatureCelsius = this.acceleratorTemperatureCelsius;
    copy.intakeSensor = this.intakeSensor;
    copy.middleSensor = this.middleSensor;
    copy.finalSensor = this.finalSensor;
    return copy;
  }
}
