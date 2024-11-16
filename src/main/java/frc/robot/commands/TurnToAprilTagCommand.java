// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class TurnToAprilTagCommand extends Command {
  private LimelightSubsystem m_subsystem;
  private DriveSubsystem m_driveSubsystem;
  private CommandJoystick m_joystick;

  PIDController turnPID = new PIDController(0.02, 0, 0);

  /** Creates a new TurnToAprilTagCommand. */
  public TurnToAprilTagCommand(LimelightSubsystem subsystem, DriveSubsystem driveSubsystem, CommandJoystick joystick) {
    m_subsystem = subsystem;
    m_driveSubsystem = driveSubsystem;
    m_joystick = joystick;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.tv == 1) {
      if (m_subsystem.m_targetAprilTagID == 0 || m_subsystem.m_targetAprilTagID == 5) {
        double speed = turnPID.calculate(m_subsystem.tx, 0);
        m_driveSubsystem.m_leftMotor.set(speed);
        m_driveSubsystem.m_rightMotor.set(-speed);
      }
      if (Math.abs(m_subsystem.tx) < LimelightConstants.turnToAprilTagAllowableError) {
        m_joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      } else {
        m_joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    m_driveSubsystem.m_leftMotor.set(0);
    m_driveSubsystem.m_rightMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
