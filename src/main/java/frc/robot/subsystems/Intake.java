// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(kIntakeCanId, kMotorType);

  /** Creates a new Intake. */
  public Intake() {
    m_motor.restoreFactoryDefaults();

    m_motor.setSmartCurrentLimit(kCurrentLimit);

    m_motor.setIdleMode(kIntakeIdleMode);
  }

  public void spin() {
    m_motor.set(kSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
