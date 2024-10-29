package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class DriveSubsystem extends SubsystemBase {
  public TalonFX m_leftMotor;
  public TalonFX m_rightMotor;

  public DriveSubsystem() {
    m_leftMotor = new TalonFX(IDConstants.leftDriveMotorID);
    m_rightMotor = new TalonFX(IDConstants.rightDriveMotorID);

    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
  }

}
