package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem {
  TalonFX m_motor = new TalonFX(0, "CANivore");

  public IntakeSubsystem() {

  }

  public void runIntake(double output) {
    m_motor.setControl(new DutyCycleOut(output));
  }

}