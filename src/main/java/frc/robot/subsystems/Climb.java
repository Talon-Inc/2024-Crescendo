// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private final CANSparkMax m_climbMotor = new CANSparkMax(ClimbConstants.kClimbCanId, ClimbConstants.kMotorType);
  private final DigitalInput limitSwitchClimbTop = new DigitalInput(ClimbConstants.kLimitSwitchTopDIOPort);
  private final DigitalInput limitSwitchClimbBottom = new DigitalInput(ClimbConstants.kLimitSwitchBottomDIOPort);

  /** Creates a new Climb. */
  public Climb() {
    // add initialization for motors
    m_climbMotor.restoreFactoryDefaults();
    m_climbMotor.setSmartCurrentLimit(ClimbConstants.kCurrentLimit);
    m_climbMotor.setIdleMode(ClimbConstants.kIdleMode);
    m_climbMotor.burnFlash();
  }

  public void climbUp() {
    m_climbMotor.set(-ClimbConstants.kSpeed);
  }

  public void climbDown() {
    m_climbMotor.set(ClimbConstants.kSpeed);
  }

  public void stopClimb() {
    m_climbMotor.set(0);
  }

  public boolean getLimitSwitchTop() {
    return limitSwitchClimbTop.get();
  }

  public boolean getLimitSwitchBottom() {
    return !limitSwitchClimbBottom.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
