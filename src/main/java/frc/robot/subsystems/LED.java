// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LED extends SubsystemBase {
  private final PWMSparkMax led = new PWMSparkMax(LedConstants.kLedChannel);

  /** Creates a new LED. */
  public LED() {}

  public void setBlack() {
    led.set(0.99);
  }

  public void setGreen() {
    led.set(0.77);
  }

  public void setGold() {
    led.set(0.67);
  }

  public void strobeGold() {
    led.set(0.35);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
