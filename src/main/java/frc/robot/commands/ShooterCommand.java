package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  private ShooterSubsystem m_subsystem;

  public ShooterCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_subsystem.runShooter(ShooterConstants.shooterSpeed);
  }

  // @Override
  public void end() {
    m_subsystem.runShooter(0.0);
    return;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
