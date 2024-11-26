// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.commands.TurnToAprilTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  DriveSubsystem m_driveSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  CANdleSubsystem m_CANdleSubsystem;
  LimelightSubsystem m_limelightSubsystem;
  CommandJoystick m_driveJoystick = new CommandJoystick(ControllerConstants.kDriverControllerPort);
  CommandJoystick m_opJoystick = new CommandJoystick(ControllerConstants.kOperatorControllerPort);

  public RobotContainer() {
    m_driveSubsystem = new DriveSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    if (SubsystemConstants.useCANdle) {
      m_CANdleSubsystem = new CANdleSubsystem(m_opJoystick, m_driveSubsystem);
    }
    m_limelightSubsystem = new LimelightSubsystem();
    configureBindings();
    NamedCommands.registerCommand("launch", new ShooterCommand(m_shooterSubsystem));
    NamedCommands.registerCommand("startIntake", new InstantCommand(() -> {
      m_intakeSubsystem.runIntake(Constants.ShooterConstants.shooterSpeed);
    }));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> {
      m_intakeSubsystem.runIntake(0);
    }));
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
      m_driveJoystick.button(ControllerConstants.shooterButtonID).whileTrue(new ShooterCommand(m_shooterSubsystem));
    }
    if (SubsystemConstants.useDrive) {
      DriveCommand drive = new DriveCommand(m_driveSubsystem, () -> {
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverLeftAxis), 2)
            * Math.signum(m_driveJoystick.getRawAxis(ControllerConstants.DriverLeftAxis));
      }, () -> {
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverRightAxis), 2)
            * Math.signum(m_driveJoystick.getRawAxis(ControllerConstants.DriverLeftAxis));
      });
      m_driveSubsystem.setDefaultCommand(drive);
      // Turn to april tag button
      m_driveJoystick.button(ControllerConstants.turnToAprilTagButtonID)
          .whileTrue(new TurnToAprilTagCommand(m_limelightSubsystem, m_driveSubsystem, m_opJoystick));
    }

    if (SubsystemConstants.useCANdle) {
      m_opJoystick.button(5).whileTrue(new RunCommand(m_CANdleSubsystem::incrementAnimation, m_CANdleSubsystem));
      m_opJoystick.button(6).whileTrue(new RunCommand(m_CANdleSubsystem::decrementAnimation, m_CANdleSubsystem));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
