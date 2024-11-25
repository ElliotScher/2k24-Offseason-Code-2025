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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Intake intake;
  private Vision vision;
  private Shooter shooter;
  private Arm arm;
  private Leds leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case WHIPLASH:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          intake = new Intake(new IntakeIOTalonFX());
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          shooter = new Shooter(new ShooterIOTalonFX());
          arm = new Arm(new ArmIOTalonFX());
          leds = new Leds();
          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (vision == null) {
      vision =
          new Vision(
              CameraConstants.ReplayCameras.LEFT_CAMERA,
              CameraConstants.ReplayCameras.RIGHT_CAMERA);
    }
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    if (leds == null) {
      leds = new Leds();
    }

    // Auto modes
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption("Drive Characterization", DriveCommands.runCharacterization(drive));
    autoChooser.addOption("Arm Characterization", arm.runCharacterization());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driver.rightBumper(),
            driver.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05),
            driver.b()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
    driver.leftBumper().whileTrue(CompositeCommands.collect(intake, arm));
    driver.leftTrigger().whileTrue(CompositeCommands.eject(intake, arm));
    driver
        .rightBumper()
        .whileTrue(CompositeCommands.shootSpeaker(drive, intake, arm, shooter, driver.getHID()));
    driver.x().whileTrue(CompositeCommands.shootSubwoofer(intake, arm, shooter));
    driver
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.95)
        .whileTrue(CompositeCommands.shootAmp(intake, arm, shooter));
    driver
        .axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05)
        .whileTrue(arm.preAmpAngle());
    driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.resetRobotPose(
                            new Pose2d(
                                FieldConstants.Subwoofer.centerFace.getTranslation(),
                                RobotState.getRobotPose().getRotation())))
                .ignoringDisable(true));
    driver.a().whileTrue(CompositeCommands.shootFeed(intake, arm, shooter));
    operator.leftBumper().whileTrue(CompositeCommands.collect(intake, arm));
  }

  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        vision.getCameras(),
        intake.hasNoteLocked(),
        intake.hasNoteStaged(),
        intake.isIntaking());
    leds.periodic();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
