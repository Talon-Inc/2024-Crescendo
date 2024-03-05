// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeCanId, IntakeConstants.kMotorType);
  private final CANSparkMax m_intakeMotor2 = new CANSparkMax(IntakeConstants.kMiddleIntakeCanId, IntakeConstants.kMotorType);
  private final DigitalInput m_intakeSensor = new DigitalInput(IntakeConstants.kSensorDIO);

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor1.restoreFactoryDefaults();
    m_intakeMotor2.restoreFactoryDefaults();

    m_intakeMotor1.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_intakeMotor2.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);

    m_intakeMotor1.setIdleMode(IntakeConstants.kIntakeIdleMode);
    m_intakeMotor2.setIdleMode(IntakeConstants.kIntakeIdleMode);

    m_intakeMotor1.burnFlash();
    m_intakeMotor2.burnFlash();
  }

  public boolean isNoteLoaded() {
    return m_intakeSensor.get();
  }

  public void intakeNote() {
    m_intakeMotor1.set(IntakeConstants.kSpeed);
    m_intakeMotor2.set(IntakeConstants.kSpeed);
  }

  public void moveIntakeChannel() {
    m_intakeMotor2.set(IntakeConstants.kSpeed);
  }

  public void outtakeNote() {
    m_intakeMotor1.set(-IntakeConstants.kSpeed);
    m_intakeMotor2.set(-IntakeConstants.kSpeed);
  }

  public void stop() {
    m_intakeMotor1.set(0);
    m_intakeMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is Note Loaded", isNoteLoaded());
  }
}
