package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
}