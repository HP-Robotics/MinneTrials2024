package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {

  public static class IDConstants {
    public static final int leftDriveMotorID = 31;
    public static final int rightDriveMotorID = 12;
    public static final int intakeMotorID = 30;
    public static final int topMotorID = 3;
    public static final int bottomMotorID = 4;
    public static final int pigeonID = 57;
    public static final int CANdleID = 50;
  }

  public static class ControllerConstants {
    public static final int kOperatorControllerPort = 1;
    public static final int kDriverControllerPort = 0;

    public static final int DriverLeftAxis = 1;
    public static final int DriverRightAxis = 5;

    public static final int intakeAxisID = 4; // right trigger
    public static final int yuckButtonID = 4;
    public static final int driveButtonID = 1;
    public static final int shooterButtonID = 1;
    public static final int turnToAprilTagButtonID = 2; // idk what button
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = false;

    public static final boolean useIntake = true;
    public static final boolean useShooter = true;
    public static final boolean useCANdle = false;
  }

  public static class IntakeConstants {
    public static final double intakeSpeed = 0.1;
  }

  public static class ShooterConstants {
    public static final double shooterSpeed = 0.1;
  }

  public static class DriveConstants {
    public static final double kEncoderResolution = 5;
    public static final double kWheelRadius = 5;
    public static final double driveGearRatio = 5;
    public static final double slowSpeedMultiplier = 0.3;
  }

  public static class LimelightConstants {
    public static final double turnToAprilTagAllowableError = 5; // For testing
  }
}