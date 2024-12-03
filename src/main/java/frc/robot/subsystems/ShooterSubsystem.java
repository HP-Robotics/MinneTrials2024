// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  TalonFX m_topMotor = new TalonFX(IDConstants.topMotorID);
  TalonFX m_bottomMotor = new TalonFX(IDConstants.bottomMotorID);

  public ShooterSubsystem() {
    m_topMotor.setInverted(true);
    m_bottomMotor.setInverted(false);
  }

  public void runShooter(double output) {
    m_topMotor.setControl(new DutyCycleOut(output));
    m_bottomMotor.setControl(new DutyCycleOut(output));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
