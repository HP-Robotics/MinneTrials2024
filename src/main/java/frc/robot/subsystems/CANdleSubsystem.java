// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IDConstants;

public class CANdleSubsystem extends SubsystemBase {
  /** Creates a new CANdleSubsystem. */
  public CANdle m_candle = new CANdle(IDConstants.CANdleID, "rio");
  public int LedCount = 307;
  public CommandJoystick joystick;
  public Animation m_toAnimate = null;
  public DriveSubsystem m_driveSubsystem;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  public AnimationTypes m_currentAnimation;

  public void incrementAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Fire:
        changeAnimation(AnimationTypes.Larson);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.SetAll);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
    }
  }

  public void decrementAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.SetAll);
        break;
      case Fire:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.Larson);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
    }
  }

  public void setColors() {
    changeAnimation(AnimationTypes.SetAll);
  }

  public void changeAnimation(AnimationTypes toChange) {
    m_currentAnimation = toChange;
    switch (toChange) {
      case ColorFlow:
        m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
        break;
      case Fire:
        m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
        break;
      case Larson:
        m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
        break;
      case Rainbow:
        m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
        break;
      case RgbFade:
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
        break;
      case SingleFade:
        m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
        break;
      case Strobe:
        m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
        break;
      case Twinkle:
        m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
        break;
      case TwinkleOff:
        m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
        break;
      case SetAll:
        m_toAnimate = null;
        break;
    }
  }

  public CANdleSubsystem(CommandJoystick joy, DriveSubsystem driveSubsystem) {
    this.joystick = joy;
    this.m_driveSubsystem = driveSubsystem;
    changeAnimation(AnimationTypes.SetAll);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    configAll.v5Enabled = true;
    m_candle.configAllSettings(configAll, 100);
    m_candle.setLEDs(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_toAnimate == null) {
      m_candle.setLEDs(
          (int) (0), 255, 255, 0,
          (int) (Math.abs(m_driveSubsystem.m_rightVelocity) * 9),
          (int) (Math.abs(m_driveSubsystem.m_leftVelocity) * 134));
      m_candle.animate(m_toAnimate);
    }
    m_candle.setLEDs(
        (int) (0), 0, 0, 0,
        (int) (Math.abs(m_driveSubsystem.m_leftVelocity) * 134),
        (67));
    m_candle.animate(m_toAnimate);
    m_candle.modulateVBatOutput(Math.abs(joystick.getRawAxis(1)));
  }
}