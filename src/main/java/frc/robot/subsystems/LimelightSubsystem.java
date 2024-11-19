// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class LimelightSubsystem extends SubsystemBase {
  DriveSubsystem m_driveSubsystem;
  NetworkTableEntry botpose_blue;
  CommandJoystick m_driveJoystick;

  public final Field2d m_field = new Field2d();
  NetworkTable m_table;
  public double tx;
  public double ty;
  public double tz;
  public double rx;
  public double ry;
  public double rz;
  public double tv;
  public int m_targetAprilTagID;

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
    tx = m_table.getEntry("tx").getDouble(0);
    ty = m_table.getEntry("ty").getDouble(0);
    tv = m_table.getEntry("tv").getDouble(0);

    Pose2d m_robotPose = new Pose2d(tx, ty, new Rotation2d());

    m_field.setRobotPose(m_robotPose);

    // TODO: Test if default value works (ID 6 isn't on field but in april tags set)
    m_targetAprilTagID = (int) m_table.getEntry("tid").getInteger(6);
  }
}
