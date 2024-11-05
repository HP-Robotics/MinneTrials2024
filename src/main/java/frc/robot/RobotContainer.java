// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  DriveSubsystem m_driveSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  LimelightSubsystem m_limelightSubsystem;
  CommandJoystick m_driveJoystick = new CommandJoystick(ControllerConstants.kDriverControllerPort);
  CommandJoystick m_opJoystick = new CommandJoystick(ControllerConstants.kOperatorControllerPort);

  public RobotContainer() {
    m_driveSubsystem = new DriveSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_limelightSubsystem = new LimelightSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    if (SubsystemConstants.useIntake) {
      // Intake button
      m_driveJoystick.axisGreaterThan(ControllerConstants.intakeAxisID, 0.1)
          .whileTrue(new IntakeCommand(m_intakeSubsystem));
      // Yuck button
      m_driveJoystick.button(ControllerConstants.yuckButtonID).whileTrue(m_intakeSubsystem.yuckCommand());
    }
    if (SubsystemConstants.useShooter) {
      // Shooter button
      m_opJoystick.button(ControllerConstants.shooterButtonID).whileTrue(new ShooterCommand(m_shooterSubsystem));
    }
    if (SubsystemConstants.useDrive) {
      DriveCommand drive = new DriveCommand(m_driveSubsystem, () -> {
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverLeftAxis), 3);
      }, () -> {
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverRightAxis), 3);
      });
      m_driveSubsystem.setDefaultCommand(drive);
      // Turn to april tag button
      m_driveJoystick.button(ControllerConstants.turnToAprilTagButtonID)
          .whileTrue(m_limelightSubsystem.turnToAprilTagCommand(m_limelightSubsystem.rx));
    }

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
