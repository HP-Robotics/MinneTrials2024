// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.fireCommandDotEdu;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.commands.TurnToAprilTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.IntakeCommand;

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
      m_driveJoystick.button(3).whileTrue(new fireCommandDotEdu(m_intakeSubsystem)); // TODO: Make it a constant
    }
    if (SubsystemConstants.useShooter) {
      // Shooter button
      // TODO: change if beambreak exists
      m_driveJoystick.button(ControllerConstants.shooterButtonID).onTrue(new SequentialCommandGroup(
          m_intakeSubsystem.yuckCommand().withTimeout(0.1),
          new ParallelCommandGroup(
              new ShooterCommand(m_shooterSubsystem).withTimeout(1.5),
              new SequentialCommandGroup(
                  new WaitCommand(0.5),
                  new IntakeCommand(m_intakeSubsystem).withTimeout(0.2)))));
    }
    if (SubsystemConstants.useDrive) {
      DriveCommand drive = new DriveCommand(m_driveSubsystem, () -> {
        // TODO: Make the 0.5 a constant
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverLeftAxis), 2)
            * Math.signum(m_driveJoystick.getRawAxis(ControllerConstants.DriverLeftAxis));
      }, () -> {
        return 0.5 * Math.pow(m_driveJoystick.getRawAxis(ControllerConstants.DriverRightAxis), 2)
            * Math.signum(m_driveJoystick.getRawAxis(ControllerConstants.DriverRightAxis));
      });
      m_driveSubsystem.setDefaultCommand(drive);
      // Turn to april tag button
      m_driveJoystick.button(ControllerConstants.turnToAprilTagButtonID)
          .whileTrue(new TurnToAprilTagCommand(m_limelightSubsystem, m_driveSubsystem, m_opJoystick));
      m_driveJoystick.button(6).onTrue(new InstantCommand(() -> {
        m_driveSubsystem.m_slowMode = !m_driveSubsystem.m_slowMode;
      }));
      m_driveJoystick.button(ControllerConstants.invertMotorsButtonID).onTrue(new InstantCommand(() -> {
        m_driveSubsystem.m_leftMotor.setInverted(false);
        m_driveSubsystem.m_rightMotor.setInverted(true);
      }));
    }

    if (SubsystemConstants.useCANdle) {
      m_opJoystick.button(5).whileTrue(new RunCommand(m_CANdleSubsystem::incrementAnimation, m_CANdleSubsystem));
      m_opJoystick.button(11).whileTrue(new RunCommand(m_CANdleSubsystem::decrementAnimation, m_CANdleSubsystem));
      m_opJoystick.button(7).onTrue(m_CANdleSubsystem.IntakeLights());
      m_opJoystick.button(7).onFalse(m_CANdleSubsystem.LightsOut());
    }
  }

  public Command getAutonomousCommand() {
    return Autos.DriveForwardShoot(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem);
  }
}
