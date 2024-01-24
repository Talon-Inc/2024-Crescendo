// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


public class LED extends SubsystemBase {
  private final PWMSparkMax led = new PWMSparkMax(LED_CHANNEL); 
  private double color = 0;

  /** Creates a new LED. */
  public LED() {}

  public void setGreen() {
    color = 0.77;
    led.set(color);
  }

  public void setBlack() {
    color = 0.99;
    led.set(color);
  }

  public double color() {
    return color;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
