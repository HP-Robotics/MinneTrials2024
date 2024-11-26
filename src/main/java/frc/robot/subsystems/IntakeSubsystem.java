package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.BeamBreak;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_motor = new TalonFX(IDConstants.intakeMotorID);
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = inst.getTable("intake-table");
  public BeamBreak m_beamBreak;

  public IntakeSubsystem() {
    intakeTable.getEntry("Intake Setpoint").getDouble(IntakeConstants.intakeSpeed);
    m_motor.setInverted(false);

    m_beamBreak = new BeamBreak(1);
  }

  public void periodic() {
    intakeTable.putValue("Intake On", NetworkTableValue.makeBoolean(true));
    intakeTable.putValue("Yucking", NetworkTableValue.makeBoolean(true));
  }

  public void yuck(double output) {
    m_motor.setControl(new DutyCycleOut(-output));
  }

  public void runIntake(double output) {
    m_motor.setControl(new DutyCycleOut(output));
  }

  public Command yuckCommand() {
    return (new StartEndCommand(
        () -> this.yuck(IntakeConstants.intakeSpeed),
        () -> this.yuck(0),
        this));
  }
}