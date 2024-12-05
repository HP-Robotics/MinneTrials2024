package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Autos {
    DriveSubsystem m_driveSubsystem;

    public Command LeftOneHoop(IntakeSubsystem m_intakeSubsystem, ShooterSubsystem m_shooterSubsystem) {
        return new SequentialCommandGroup(
                m_driveSubsystem.followPathCommand("LeftOneHoop Part1"),
                m_driveSubsystem.followPathCommand("LeftOneHoop Part2"),
                new ShooterCommand(m_shooterSubsystem));

    }
    public static Command DriveForwardShoot(DriveSubsystem m_driveSubsystem, ShooterSubsystem m_shooterSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_driveSubsystem.driveStraight(), m_driveSubsystem),
            new WaitCommand(3).withTimeout(2.0),
            new ShooterCommand(m_shooterSubsystem));
    }
}