// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  DriveSubsystem m_driveSubsystem;
  CommandJoystick m_driveJoystick = new CommandJoystick(ControllerConstants.kDriverControllerPort);

  public RobotContainer() {
    m_driveSubsystem = new DriveSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    if (SubsystemConstants.useIntake) {
      m_driveJoystick.button(ControllerConstants.intakeButtonID).whileTrue(new IntakeCommand());
    }
    if (SubsystemConstants.useShooter) {
      m_driveJoystick.button(ControllerConstants.shooterButtonID).whileTrue(new ShooterCommand());
    }
    if (SubsystemConstants.useDrive) {
      DriveCommand drive = new DriveCommand(m_driveSubsystem, () -> {
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverLeftAxis), 3);
      }, () -> {
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverRightAxis), 3);
      });
      m_driveSubsystem.setDefaultCommand(drive);
    }

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
