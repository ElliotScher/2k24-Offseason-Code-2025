package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CameraIOInputsAutoLogged extends CameraIO.CameraIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("XOffset", xOffset);
    table.put("YOffset", yOffset);
    table.put("TargetAquired", targetAquired);
    table.put("TotalTargets", totalTargets);
    table.put("AverageDistance", averageDistance);
    table.put("FrameTimestamp", frameTimestamp);
    table.put("PrimaryPose", primaryPose);
    table.put("SecondaryPose", secondaryPose);
  }

  @Override
  public void fromLog(LogTable table) {
    xOffset = table.get("XOffset", xOffset);
    yOffset = table.get("YOffset", yOffset);
    targetAquired = table.get("TargetAquired", targetAquired);
    totalTargets = table.get("TotalTargets", totalTargets);
    averageDistance = table.get("AverageDistance", averageDistance);
    frameTimestamp = table.get("FrameTimestamp", frameTimestamp);
    primaryPose = table.get("PrimaryPose", primaryPose);
    secondaryPose = table.get("SecondaryPose", secondaryPose);
  }

  public CameraIOInputsAutoLogged clone() {
    CameraIOInputsAutoLogged copy = new CameraIOInputsAutoLogged();
    copy.xOffset = this.xOffset;
    copy.yOffset = this.yOffset;
    copy.targetAquired = this.targetAquired;
    copy.totalTargets = this.totalTargets;
    copy.averageDistance = this.averageDistance;
    copy.frameTimestamp = this.frameTimestamp;
    copy.primaryPose = this.primaryPose;
    copy.secondaryPose = this.secondaryPose;
    return copy;
  }
}
