package frc.robot;

public final class Constants {

  public static class IDConstants {
    public static final int leftDriveMotorID = 11;
    public static final int rightDriveMotorID = 12;
    public static final int intakeMotorID = 31;
    public static final int shooterMotorID = 3;
  }

  public static class ControllerConstants {
    public static final int kOperatorControllerPort = 1;
    public static final int kDriverControllerPort = 0;

    public static final int DriverLeftAxis = 1;
    public static final int DriverRightAxis = 5;

    public static final int intakeAxisID = 3; // right trigger
    public static final int yuckButtonID = 4;
    public static final int driveButtonID = 1;
    public static final int shooterButtonID = 1;
    public static final int turnToAprilTagButtonID = 0; // idk what button
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = false;
    public static final boolean useIntake = true;
    public static final boolean useShooter = false;
  }

  public static class IntakeConstants {
    public static final double intakeSpeed = 0.1;
  }

  public static class ShooterConstants {
    public static final double shooterSpeed = 0.1;
  }

  public static class LimelightConstants {
    public static final double turnToAprilTagAllowableError = 0.03;
  }
}