package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIOInputsAutoLogged inputs;
  private final ShooterIO io;
  private double topVelocitySetPointRadiansPerSecond;
  private double bottomVelocitySetPointRadiansPerSecond;

  private boolean isClosedLoop;
  private final SysIdRoutine characterizationRoutine;

  public Shooter(ShooterIO io) {

    inputs = new ShooterIOInputsAutoLogged();
    this.io = io;
    topVelocitySetPointRadiansPerSecond = 0.0;
    isClosedLoop = true;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Shooter/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));
  }

  /**
   * Performs periodic updates for the shooter subsystem. This method is called every 20ms by the
   * WPILib framework. It updates the input values from the shooter IO, processes the inputs for
   * logging, sets the velocity setpoint for the top and bottom motors if closed-loop control is
   * enabled, and updates the PID, feedforward, and motion profile configurations based on tunable
   * values.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if (isClosedLoop) {
      io.setTopVelocitySetPoint(topVelocitySetPointRadiansPerSecond);
      io.setBottomVelocitySetPoint(-bottomVelocitySetPointRadiansPerSecond);
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          io.setTopPID(pid[0], 0.0, pid[1]);
          io.setBottomPID(pid[0], 0.0, pid[1]);
        },
        ShooterConstants.KP,
        ShooterConstants.KD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        ff -> {
          io.setTopFeedForward(ff[0], ff[1], ff[2]);
          io.setBottomFeedForward(ff[0], ff[1], ff[2]);
        },
        ShooterConstants.KS,
        ShooterConstants.KV,
        ShooterConstants.KA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        profile -> {
          io.setTopProfile(profile[0]);
          io.setBottomProfile(profile[0]);
        },
        ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    Logger.recordOutput("Shooter/Position", inputs.topPosition.getRadians());
    Logger.recordOutput(
        "Shooter/Error", inputs.topVelocityGoalRadiansPerSec - inputs.topVelocityRadPerSec);
  }

  /**
   * Sets the shooter's velocity to the dynamic feed speed. This command will run once and set the
   * velocity setpoint to the feed speed obtained from the {@link RobotState#getControlData()}
   * method. It will also enable the closed-loop control of the shooter motors.
   *
   * @return a command to set the shooter's velocity to the feed speed
   */
  public Command setFeedVelocity() {

    return Commands.runOnce(
        () -> {
          topVelocitySetPointRadiansPerSecond = ShooterConstants.FEED_SPEED.get();
          bottomVelocitySetPointRadiansPerSecond = topVelocitySetPointRadiansPerSecond;
          isClosedLoop = true;
        });
  }

  /**
   * Sets the shooter's velocity to the pre-defined amplifier speed. This command will run once and
   * set the velocity setpoint to the amplifier speed. It will also enable the closed-loop control
   * of the shooter motors.
   *
   * @return a command to set the shooter's velocity to the amplifier speed
   */
  public Command setAmpVelocity() {

    return Commands.runOnce(
        () -> {
          topVelocitySetPointRadiansPerSecond = ShooterConstants.TOP_AMP_SPEED.get();
          bottomVelocitySetPointRadiansPerSecond = ShooterConstants.BOTTOM_AMP_SPEED.get();
          isClosedLoop = true;
        });
  }

  /**
   * Sets the shooter's velocity to the pre-defined subwoofer speed. This command will run once and
   * set the velocity setpoint to the subwoofer speed. It will also enable the closed-loop control
   * of the shooter motors.
   *
   * @return a command to set the shooter's velocity to the subwoofer speed
   */
  public Command setSpeakerVelocity() {
    return Commands.runOnce(
        () -> {
          topVelocitySetPointRadiansPerSecond = ShooterConstants.SPEAKER_SPEED.get();
          bottomVelocitySetPointRadiansPerSecond = topVelocitySetPointRadiansPerSecond;
          isClosedLoop = true;
        });
  }

  /**
   * Checks if the shooter motors are at the velocity setpoint. This method is used to determine if
   * the shooter is at the desired speed.
   *
   * @return true if the shooter motors are at the velocity setpoint, false otherwise. The method
   *     returns the result of the {@link ShooterIO#atSetPoint()} method.
   */
  @AutoLogOutput(key = "Shooter/at setpoint")
  public boolean atSetPoint() {
    return io.atSetPoint();
  }

  /**
   * Runs quasistatic and dynamic tests with the motors moving both forwards and backwards to
   * calculate the feedForward gains.
   *
   * @return the feedFoward gains calculated by the tests
   */
  public Command runCharacterization() {

    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(5),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(5),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(5),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
