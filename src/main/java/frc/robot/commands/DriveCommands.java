// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public final class DriveCommands {
  @Getter
  private static PIDController aimController =
      new PIDController(
          DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp(),
          0,
          DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd(),
          Constants.LOOP_PERIOD_SECONDS);

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static final Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier speakerAim,
      BooleanSupplier ampAim,
      BooleanSupplier feedAim) {
    return Commands.run(
        () -> {
          aimController.enableContinuousInput(-Math.PI, Math.PI);

          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  DriveConstants.DRIVER_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DRIVER_DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Get robot relative vel
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          double robotRelativeXVel =
              linearVelocity.getX() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
          double robotRelativeYVel =
              linearVelocity.getY() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity();

          double angular = 0.0;

          if (speakerAim.getAsBoolean()) {
            angular =
                RobotState.getControlData().speakerRadialVelocity()
                    + (aimController.calculate(
                        RobotState.getRobotPose().getRotation().getRadians(),
                        RobotState.getControlData().speakerRobotAngle().getRadians()));
          } else if (ampAim.getAsBoolean()) {
            angular =
                RobotState.getControlData().speakerRadialVelocity()
                    + (aimController.calculate(
                        RobotState.getRobotPose().getRotation().getRadians(),
                        Rotation2d.fromDegrees(90.0).getRadians()));
          } else if (feedAim.getAsBoolean()) {
            angular =
                RobotState.getControlData().speakerRadialVelocity()
                    + (aimController.calculate(
                        RobotState.getRobotPose().getRotation().getRadians(),
                        Rotation2d.fromDegrees(-35.5).getRadians()));
          } else {
            angular = omega * DriveConstants.DRIVE_CONFIG.maxAngularVelocity();
          }

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  robotRelativeXVel,
                  robotRelativeYVel,
                  angular,
                  isFlipped
                      ? RobotState.getRobotPose().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getRobotPose().getRotation());

          // Convert to field relative speeds & send command
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static final Command aimTowardSpeaker(Drive drive) {
    aimController.setTolerance(Units.degreesToRadians(1.0));
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              ChassisSpeeds chassisSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      0,
                      0,
                      RobotState.getControlData().speakerRadialVelocity()
                          + (aimController.calculate(
                              RobotState.getRobotPose().getRotation().getRadians(),
                              RobotState.getControlData().speakerRobotAngle().getRadians())),
                      isFlipped
                          ? RobotState.getRobotPose().getRotation().plus(new Rotation2d(Math.PI))
                          : RobotState.getRobotPose().getRotation());

              // Convert to field relative speeds & send command
              drive.runVelocity(chassisSpeeds);
            },
            drive)
        .finallyDo(
            () -> {
              drive.stop();
            });
  }

  public static final Command runFFCharacterization(Drive drive) {
    return new KSCharacterization(
        drive, drive::runCharacterization, drive::getFFCharacterizationVelocity);
  }

  public static final Command wheelRadiusCharacterization(Drive drive) {
    double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.runOnce(
            () -> {
              limiter.reset(0.0);
            })
        .andThen(
            Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                    // Reset acceleration limiter
                    Commands.runOnce(
                        () -> {
                          limiter.reset(0.0);
                        }),

                    // Turn in place, accelerating up to full speed
                    Commands.run(
                        () -> {
                          double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                          drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                        },
                        drive)),

                // Measurement sequence
                Commands.sequence(
                    // Wait for modules to fully orient before starting measurement
                    Commands.waitSeconds(1.0),

                    // Record starting measurement
                    Commands.runOnce(
                        () -> {
                          state.positions = drive.getWheelRadiusCharacterizationPositions();
                          state.lastAngle = drive.getRawGyroRotation();
                          state.gyroDelta = 0.0;
                        }),

                    // Update gyro delta
                    Commands.run(
                            () -> {
                              var rotation = drive.getRawGyroRotation();
                              state.gyroDelta +=
                                  Math.abs(rotation.minus(state.lastAngle).getRadians());
                              state.lastAngle = rotation;
                            })

                        // When cancelled, calculate and print results
                        .finallyDo(
                            () -> {
                              double[] positions = drive.getWheelRadiusCharacterizationPositions();
                              double wheelDelta = 0.0;
                              for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                              }
                              double wheelRadius =
                                  (state.gyroDelta * DriveConstants.DRIVE_CONFIG.driveBaseRadius())
                                      / wheelDelta;

                              NumberFormat formatter = new DecimalFormat("#0.000");
                              System.out.println(
                                  "********** Wheel Radius Characterization Results **********");
                              System.out.println(
                                  "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                              System.out.println(
                                  "\tGyro Delta: "
                                      + formatter.format(state.gyroDelta)
                                      + " radians");
                              System.out.println(
                                  "\tWheel Radius: "
                                      + formatter.format(wheelRadius)
                                      + " meters, "
                                      + formatter.format(Units.metersToInches(wheelRadius))
                                      + " inches");
                            }))));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
