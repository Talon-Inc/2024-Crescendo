// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor1 = new CANSparkMax(kIntakeCanId1, kMotorType);
  private final CANSparkMax m_intakeMotor2 = new CANSparkMax(kIntakeCanId2, kMotorType);
  private final DigitalInput m_intakeSensor = new DigitalInput(0);

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor1.restoreFactoryDefaults();
    m_intakeMotor2.restoreFactoryDefaults();

    m_intakeMotor1.setSmartCurrentLimit(kCurrentLimit);
    m_intakeMotor2.setSmartCurrentLimit(kCurrentLimit);

    m_intakeMotor1.setIdleMode(kIntakeIdleMode);
    m_intakeMotor2.setIdleMode(kIntakeIdleMode);
  }

  public boolean isNoteLoaded() {
    return m_intakeSensor.get();
  }

  public void intakeNote() {
    m_intakeMotor1.set(kSpeed);
    m_intakeMotor2.set(kSpeed);
  }
  public void outtakeNote(){
    m_intakeMotor1.set(-kSpeed);
    m_intakeMotor2.set(-kSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
