package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private IntakeSubsystem m_subsystem;

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_subsystem.runIntake(IntakeConstants.intakeSpeed);
  }

  // @Override
  public void end() {
    m_subsystem.runIntake(0.0);
    return;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
