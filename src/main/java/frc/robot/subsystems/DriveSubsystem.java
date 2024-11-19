package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPLTVController;

public class DriveSubsystem extends SubsystemBase {
  public TalonFX m_leftMotor;
  // public TalonFX m_rightMotor;
  public ReplanningConfig m_config;
  public DifferentialDriveOdometry m_odometry;
  private final Pigeon2 m_gyro = new Pigeon2(IDConstants.pigeonID, "CANivore");
  public double leftVelocity = 0.0; // left velocity
  public double rightVelocity = 0.0; // right velocity

  public DriveSubsystem() {
    m_leftMotor = new TalonFX(IDConstants.leftDriveMotorID);
    // m_rightMotor = new TalonFX(IDConstants.rightDriveMotorID);

    m_leftMotor.setInverted(false);
    // m_rightMotor.setInverted(true);
    try {
      m_config = new ReplanningConfig(true, true);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    /*
     * DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
     * m_gyro.getRotation2d(),
     * ticksToMeters(m_leftMotor.getRotorPosition().getValue()),
     * ticksToMeters(m_rightMotor.getRotorPosition().getValue()),
     * new Pose2d(5.0, 13.5, new Rotation2d()));
     */
  }

  public Command followPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      return new FollowPathCommand(
          path,
          this::getPose,
          this::getRobotRelativeSpeeds,
          this::drive,
          new PPLTVController(0.2),
          m_config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    leftVelocity = wheelSpeeds.leftMetersPerSecond; // left velocity
    rightVelocity = wheelSpeeds.rightMetersPerSecond; // right velocity
    m_leftMotor.set(leftVelocity);
    // m_rightMotor.set(rightVelocity);
  }

  public void drive(double Left, double Right) {
    leftVelocity = Left;
    rightVelocity = Right;
    m_leftMotor.set(leftVelocity);
  }

  // encoder speed to meters per second
  public double ticksToMeters(double ticks) {
    return ((ticks / DriveConstants.kEncoderResolution) * (2 * Math.PI * DriveConstants.kWheelRadius))
        / DriveConstants.driveGearRatio;
  }

  public double metersToTicks(double meters) {
    return (meters / (2 * Math.PI * DriveConstants.kWheelRadius)) * DriveConstants.kEncoderResolution
        * DriveConstants.driveGearRatio;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return null;
  }

}