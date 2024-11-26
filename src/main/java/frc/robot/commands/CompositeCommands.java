package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
  public static final Command resetHeading(Drive drive) {
    return Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(
                  new Pose2d(
                      RobotState.getRobotPose().getTranslation(),
                      AllianceFlipUtil.apply(new Rotation2d())));
            })
        .ignoringDisable(true);
  }

  public static final Command collect(Intake intake, Arm arm) {
    return Commands.sequence(
        arm.intakeAngle(), Commands.waitUntil(() -> arm.atSetpoint()), intake.intake());
  }

  public static final Command eject(Intake intake, Arm arm) {
    return Commands.sequence(
        arm.ejectCommand(), Commands.waitUntil(() -> arm.atSetpoint()), intake.eject());
  }

  public static final Command shootSpeaker(
      Drive drive, Intake intake, Arm arm, Shooter shooter, XboxController driver) {
    return Commands.sequence(
            Commands.parallel(shooter.setSpeakerVelocity(), arm.shootAngle())
                .until(
                    () ->
                        shooter.atSetPoint()
                            && arm.atSetpoint()
                            && DriveCommands.atAimSetpoint()
                            && drive.getYawVelocity() <= Units.degreesToRadians(1)),
            Commands.waitSeconds(0.125),
            intake.shoot(),
            Commands.either(
                Commands.sequence(
                    intake.intake(),
                    Commands.parallel(shooter.setSpeakerVelocity(), arm.shootAngle())
                        .until(
                            () ->
                                shooter.atSetPoint()
                                    && arm.atSetpoint()
                                    && DriveCommands.atAimSetpoint()),
                    Commands.waitSeconds(0.125),
                    intake.shoot(),
                    arm.stowAngle()),
                arm.stowAngle(),
                () -> intake.hasNoteStaged()),
            Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 1)),
            Commands.waitSeconds(1))
        .finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0));
  }

  public static final Command shootSpeakerAuto(
      Drive drive, Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.deadline(shooter.setSpeakerVelocity(), arm.shootAngle()),
        Commands.waitUntil(
            () -> shooter.atSetPoint() && arm.atSetpoint() && DriveCommands.atAimSetpoint()),
        Commands.runOnce(() -> drive.stop()),
        Commands.waitSeconds(0.125),
        intake.shoot());
  }

  public static final Command shootSubwoofer(Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setSpeakerVelocity(), arm.subwooferAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setSpeakerVelocity(), arm.subwooferAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootAmp(Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setAmpVelocity(), arm.ampAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setAmpVelocity(), arm.ampAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }

  public static final Command shootFeed(Intake intake, Arm arm, Shooter shooter) {
    return Commands.sequence(
        Commands.parallel(shooter.setFeedVelocity(), arm.feedAngle()),
        Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
        intake.shoot(),
        Commands.either(
            Commands.sequence(
                CompositeCommands.collect(intake, arm),
                Commands.parallel(shooter.setFeedVelocity(), arm.feedAngle()),
                Commands.waitUntil(() -> shooter.atSetPoint() && arm.atSetpoint()),
                intake.shoot(),
                arm.stowAngle()),
            arm.stowAngle(),
            () -> intake.hasNoteStaged()));
  }
}
