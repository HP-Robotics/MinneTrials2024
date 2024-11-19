package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {

  public static class IDConstants {
    public static final int leftDriveMotorID = 31;
    public static final int rightDriveMotorID = 12;
    public static final int intakeMotorID = 30;
    public static final int shooterMotorID = 3;
    public static final int pigeonID = 57;
    public static final int CANdleID = 50;
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
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = true;
    public static final boolean useIntake = false;
    public static final boolean useShooter = false;
    public static final boolean useCANdle = true;
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
  }
}