// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotState;

public class Leds {
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private static final int length = 40;

  public Leds() {
    leds = new AddressableLED(9);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

  public void periodic() {
    if (RobotState.getControlData().hasNoteLocked()
        && RobotState.getControlData().hasNoteStaged()) {
      strobe(Color.kGreen, Color.kBlack, 1.0);
    } else if (RobotState.getControlData().hasNoteLocked()) {
      solid(Color.kGreen);
    } else if (RobotState.getControlData().hasNoteStaged()) {
      solid(Color.kDarkMagenta);
    } else if (RobotState.getControlData().isIntaking()) {
      solid(Color.kDarkBlue);
    } else {
      solid(Color.kBlack);
    }

    leds.setData(buffer);
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2);
  }

  private void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < length; i++) {
        buffer.setLED(i, color);
      }
    }
  }
}
