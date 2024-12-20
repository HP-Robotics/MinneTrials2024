package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Autos {
    DriveSubsystem m_driveSubsystem;
    IntakeSubsystem m_intakeSubsystem;

    public Command LeftOneHoop(IntakeSubsystem m_intakeSubsystem, ShooterSubsystem m_shooterSubsystem) {
        return new SequentialCommandGroup(
                m_driveSubsystem.followPathCommand("LeftOneHoop Part1"),
                m_driveSubsystem.followPathCommand("LeftOneHoop Part2"),
                new ShooterCommand(m_shooterSubsystem));

    }

    public static Command DriveForwardShoot(DriveSubsystem m_driveSubsystem, ShooterSubsystem m_shooterSubsystem,
            IntakeSubsystem m_intakeSubsystem) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_driveSubsystem.driveStraight(), m_driveSubsystem),
                new WaitCommand(1.0),
                new InstantCommand(() -> m_driveSubsystem.drive(0, 0), m_driveSubsystem),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            m_intakeSubsystem.yuckCommand().withTimeout(0.1);
                        }),
                        new ParallelCommandGroup(
                                new ShooterCommand(m_shooterSubsystem).withTimeout(1.5),
                                new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new IntakeCommand(m_intakeSubsystem).withTimeout(0.2)))));
    }
}