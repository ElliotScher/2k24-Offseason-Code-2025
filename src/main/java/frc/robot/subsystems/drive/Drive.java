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

package frc.robot.subsystems.drive;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final Lock odometryLock;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs;
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final LinearFilter xFilter;
  private final LinearFilter yFilter;
  private double filteredX;
  private double filteredY;

  private final SwerveDriveKinematics kinematics;
  @Getter private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions;

  @Getter private AutoFactory autoFactory;

  static {
    odometryLock = new ReentrantLock();
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    gyroInputs = new GyroIOInputsAutoLogged();
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op if no signals have been created)
    PhoenixOdometryThread.getInstance().start();

    xFilter = LinearFilter.movingAverage(10);
    yFilter = LinearFilter.movingAverage(10);
    filteredX = 0;
    filteredY = 0;

    kinematics = DriveConstants.DRIVE_CONFIG.kinematics();
    lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    autoFactory =
        Choreo.createAutoFactory(
            this,
            RobotState::getRobotPose,
            new AutoController(this),
            AllianceFlipUtil::shouldFlip,
            new AutoBindings());
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
      Translation2d rawFieldRelativeVelocity =
          new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
              .rotateBy(getRawGyroRotation());

      filteredX = xFilter.calculate(rawFieldRelativeVelocity.getX());
      filteredY = yFilter.calculate(rawFieldRelativeVelocity.getY());
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds.discretize(Constants.LOOP_PERIOD_SECONDS);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.DRIVE_CONFIG.maxLinearVelocity());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i], new SwerveModuleState());
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocity and torque.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocityTorque(ChassisSpeeds speeds, List<Vector<N2>> forces) {
    if (forces.size() != 4) {
      throw new IllegalArgumentException("Forces array must have 4 elements");
    }
    // Calculate module setpoints
    speeds.discretize(Constants.LOOP_PERIOD_SECONDS);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveModuleState[] setpointTorques = new SwerveModuleState[4];
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.DRIVE_CONFIG.maxLinearVelocity());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      Vector<N2> wheelDirection =
          VecBuilder.fill(setpointStates[i].angle.getCos(), setpointStates[i].angle.getSin());
      setpointTorques[i] =
          new SwerveModuleState(
              forces.get(i).dot(wheelDirection) * DriveConstants.FRONT_LEFT.DriveMotorGearRatio,
              setpointStates[i].angle);

      setpointStates[i].optimize(modules[i].getAngle());
      setpointTorques[i].optimize(modules[i].getAngle());

      modules[i].runSetpoint(setpointStates[i], setpointTorques[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/TorquesOptimized", setpointTorques);
  }

  /** Runs the drive in a straight line with the specified drive current. */
  public void runCharacterization(double amps) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(amps);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.DRIVE_CONFIG.getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition().getRadians();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.DRIVE_CONFIG.maxLinearVelocity();
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DriveConstants.DRIVE_CONFIG.driveBaseRadius();
  }

  /** Returns the field relative velocity in X and Y. */
  public Translation2d getFieldRelativeVelocity() {
    return new Translation2d(filteredX, filteredY);
  }

  /** Returns the current yaw velocity */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  public class AutoController implements BiConsumer<Pose2d, SwerveSample> {
    private final Drive drive; // drive subsystem
    private final PIDController xController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kp(),
            0.0,
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kd());
    private final PIDController yController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kp(),
            0.0,
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kd());
    private final PIDController headingController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp(),
            0.0,
            DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd());

    public AutoController(Drive drive) {
      this.drive = drive;
      headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void accept(Pose2d pose, SwerveSample referenceState) {
      double xFF = referenceState.vx;
      double yFF = referenceState.vy;
      double rotationFF = referenceState.omega;

      double xFeedback = xController.calculate(pose.getX(), referenceState.x);
      double yFeedback = yController.calculate(pose.getY(), referenceState.y);
      double rotationFeedback =
          headingController.calculate(pose.getRotation().getRadians(), referenceState.heading);

      ChassisSpeeds velocity =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());

      List<Vector<N2>> moduleTorques = new ArrayList<>(4);

      for (int i = 0; i < 4; i++) {
        moduleTorques.add(
            new Translation2d(referenceState.moduleForcesX()[i], referenceState.moduleForcesY()[i])
                .rotateBy(Rotation2d.fromRadians(referenceState.heading))
                .unaryMinus()
                .toVector());
      }

      drive.runVelocityTorque(velocity, moduleTorques);
    }
  }
}
