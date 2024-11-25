package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class KSCharacterization extends Command {
  private static final LoggedTunableNumber currentRampFactor =
      new LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final LoggedTunableNumber minVelocity =
      new LoggedTunableNumber("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Supplier<Rotation2d> positionSupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public KSCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    this.positionSupplier = () -> Rotation2d.fromDegrees(90);
    addRequirements(subsystem);
  }

  public KSCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier,
      Supplier<Rotation2d> positionSupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    this.positionSupplier = positionSupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("********** FF Characterization Results **********");
    System.out.println(
        "Static Characterization output: "
            + currentInput * Math.sin(positionSupplier.get().getRadians())
            + " amps");
    inputConsumer.accept(0);
  }
}
