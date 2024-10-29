// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
  DriveSubsystem m_driveSubsystem;
  DoubleSupplier m_leftAxis;
  DoubleSupplier m_rightAxis;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
    m_driveSubsystem = driveSubsystem;
    m_leftAxis = leftAxis;
    m_rightAxis = rightAxis;
    addRequirements(m_driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.m_leftMotor.set(m_leftAxis.getAsDouble());
    m_driveSubsystem.m_rightMotor.set(m_rightAxis.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
