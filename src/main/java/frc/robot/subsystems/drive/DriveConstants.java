package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
  public static final DriveConfig DRIVE_CONFIG;

  public static final SwerveModuleConstants FRONT_LEFT;
  public static final SwerveModuleConstants FRONT_RIGHT;
  public static final SwerveModuleConstants BACK_LEFT;
  public static final SwerveModuleConstants BACK_RIGHT;

  public static final Gains GAINS;
  public static final AutoAlignGains AUTO_ALIGN_GAINS;

  public static final double ODOMETRY_FREQUENCY;
  public static final double DRIVER_DEADBAND;

  static {
    switch (Constants.ROBOT) {
      case WHIPLASH:
      default:
        DRIVE_CONFIG =
            new DriveConfig(
                WhiplashTunerConstants.DrivetrainConstants.CANBusName,
                WhiplashTunerConstants.DrivetrainConstants.Pigeon2Id,
                WhiplashTunerConstants.FrontLeft.WheelRadius,
                Math.abs(WhiplashTunerConstants.FrontLeft.LocationX)
                    + Math.abs(WhiplashTunerConstants.FrontRight.LocationX),
                Math.abs(WhiplashTunerConstants.FrontLeft.LocationY)
                    + Math.abs(WhiplashTunerConstants.BackLeft.LocationY),
                WhiplashTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1));

        FRONT_LEFT = WhiplashTunerConstants.FrontLeft;
        FRONT_RIGHT = WhiplashTunerConstants.FrontRight;
        BACK_LEFT = WhiplashTunerConstants.BackLeft;
        BACK_RIGHT = WhiplashTunerConstants.BackRight;

        GAINS =
            new Gains(
                WhiplashTunerConstants.FrontLeft.DriveMotorGains.kS,
                WhiplashTunerConstants.FrontLeft.DriveMotorGains.kV,
                new LoggedTunableNumber(
                    "Drive/Gains/Drive Kp", WhiplashTunerConstants.FrontLeft.DriveMotorGains.kP),
                new LoggedTunableNumber(
                    "Drive/Gains/Drive Kd", WhiplashTunerConstants.FrontLeft.DriveMotorGains.kD),
                new LoggedTunableNumber(
                    "Drive/Gains/Turn Kp", WhiplashTunerConstants.FrontLeft.SteerMotorGains.kP),
                new LoggedTunableNumber(
                    "Drive/Gains/Turn Kd", WhiplashTunerConstants.FrontLeft.SteerMotorGains.kD));

        AUTO_ALIGN_GAINS = new AutoAlignGains(4.0, 0.0, 5.0, 0.05);

        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        break;
    }
  }

  public record DriveConfig(
      String canBus,
      int pigeon2Id,
      double wheelRadius,
      double trackWidthX,
      double trackWidthY,
      double maxLinearVelocity,
      DCMotor driveModel,
      DCMotor turnModel) {
    public double driveBaseRadius() {
      return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    }

    public double maxAngularVelocity() {
      return maxLinearVelocity / driveBaseRadius();
    }

    public Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(
            WhiplashTunerConstants.FrontLeft.LocationX, WhiplashTunerConstants.FrontLeft.LocationY),
        new Translation2d(
            WhiplashTunerConstants.FrontRight.LocationX,
            WhiplashTunerConstants.FrontRight.LocationY),
        new Translation2d(
            WhiplashTunerConstants.BackLeft.LocationX, WhiplashTunerConstants.BackLeft.LocationY),
        new Translation2d(
            WhiplashTunerConstants.BackRight.LocationX, WhiplashTunerConstants.BackRight.LocationY)
      };
    }

    public SwerveDriveKinematics kinematics() {
      return new SwerveDriveKinematics(getModuleTranslations());
    }
  }

  public record Gains(
      double driveKs,
      double driveKv,
      LoggedTunableNumber driveKp,
      LoggedTunableNumber driveKd,
      LoggedTunableNumber turnKp,
      LoggedTunableNumber turnKd) {}

  public record AutoAlignGains(
      double translation_Kp, double translation_Kd, double rotation_Kp, double rotation_Kd) {}
}
