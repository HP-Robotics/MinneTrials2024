// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  DriveSubsystem m_driveSubsystem;
  NetworkTableEntry botpose_blue;

  public final Field2d m_field = new Field2d();
  NetworkTable m_table;
  public double tx = 0;
  public double ty = 0;
  public double tz = 0;
  public double rx = 0;
  public double ry = 0;
  public double rz = 0;

  PIDController turnPID = new PIDController(0.02, 0, 0);

  public LimelightSubsystem() {
    Shuffleboard.getTab("shuffleboard")
        .add("Pose2d", m_field)
        .withWidget(BuiltInWidgets.kField);
    m_table = NetworkTableInstance.getDefault().getTable("limelight-delight");
    botpose_blue = m_table.getEntry("botpose_wpiblue");
  }

  @Override
  public void periodic() {
    double defaultValues[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double[] botpose = botpose_blue.getDoubleArray(defaultValues);
    double tx = m_table.getEntry("tx").getDouble(0);
    double ty = m_table.getEntry("ty").getDouble(0);

    Pose2d m_robotPose = new Pose2d(tx, ty, new Rotation2d());

    m_field.setRobotPose(m_robotPose);
  }

  public Command turnToAprilTagCommand(double tx) {
    return new RunCommand(() -> {
      double speed = turnPID.calculate(tx, 0);
      m_driveSubsystem.m_leftMotor.set(speed);
      m_driveSubsystem.m_rightMotor.set(-speed);
    }).finallyDo(() -> {
      m_driveSubsystem.m_leftMotor.set(0);
      m_driveSubsystem.m_rightMotor.set(0);
    });
  }
}
