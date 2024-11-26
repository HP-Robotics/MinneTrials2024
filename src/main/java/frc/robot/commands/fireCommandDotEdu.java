package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class fireCommandDotEdu extends Command {
  private IntakeSubsystem m_subsystem;

  public fireCommandDotEdu(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_subsystem.runIntake(IntakeConstants.intakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.runIntake(0.0);
    return;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
