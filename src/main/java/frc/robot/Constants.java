package frc.robot;

public final class Constants {

  public static class IDConstants {
    public static final int leftDriveMotorID = 11;
    public static final int rightDriveMotorID = 12;
    public static final int intakeMotorID = 31;
    public static final int topMotorID = 21;
    public static final int bottomMotorID = 22;
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
    public static final int turnToAprilTagButtonID = 10;
    public static final int invertMotorsButtonID = 9;
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = true;

    public static final boolean useIntake = true;
    public static final boolean useShooter = true;
    public static final boolean useCANdle = false;
  }

  public static class IntakeConstants {
    public static final double intakeSpeed = 0.5;
  }

  public static class ShooterConstants {
    public static final double shooterSpeed = 0.45;
  }

  public static class DriveConstants {
    public static final double kEncoderResolution = 5;
    public static final double kWheelRadius = 5;
    public static final double driveGearRatio = 5;
    public static final double slowSpeedMultiplier = 0.15;
    public static final double autoSpeed = 0.5;
  }

  public static class LimelightConstants {
    public static final double turnToAprilTagAllowableError = 5; // For testing
  }
}