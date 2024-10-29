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
  TalonFX m_motor = new TalonFX(IDConstants.shooterMotorID);

  public ShooterSubsystem() {
  }

  public void runShooter(double output) {
    m_motor.setControl(new DutyCycleOut(output));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
