// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shootMotorTop = new CANSparkMax(ShooterConstants.m_ShootMotorTopCanID, ShooterConstants.kMotorType);
  private final CANSparkMax m_shootMotorBottom = new CANSparkMax(ShooterConstants.m_ShootMotorBottomCanID, ShooterConstants.kMotorType);
  private final RelativeEncoder m_encoderTop = m_shootMotorTop.getEncoder();
  private final RelativeEncoder m_encoderBottom = m_shootMotorBottom.getEncoder();
  private final SparkPIDController m_pidControllerTop = m_shootMotorTop.getPIDController();
  private final SparkPIDController m_pidControllerBottom = m_shootMotorBottom.getPIDController();

  /** Creates a new ShooterArm. */
  public Shooter() {
    m_shootMotorTop.restoreFactoryDefaults();
    m_shootMotorBottom.restoreFactoryDefaults();

    m_pidControllerTop.setFeedbackDevice(m_encoderTop);
    m_pidControllerBottom.setFeedbackDevice(m_encoderBottom);

    m_shootMotorTop.setSmartCurrentLimit(ShooterConstants.kShooterSmartCurrentLimit);
    m_shootMotorBottom.setSmartCurrentLimit(ShooterConstants.kShooterSmartCurrentLimit);

    m_shootMotorTop.setIdleMode(ShooterConstants.kShooterIdleMode);
    m_shootMotorBottom.setIdleMode(ShooterConstants.kShooterIdleMode);

    // Invert bottom motor
    m_shootMotorBottom.setInverted(true);

    // PID for top motor
    m_pidControllerTop.setP(ShooterConstants.kShooterP[0]);
    m_pidControllerTop.setI(ShooterConstants.kShooterI[0]);
    m_pidControllerTop.setD(ShooterConstants.kShooterD[0]);
    m_pidControllerTop.setFF(ShooterConstants.kShooterFF[0]);
    m_pidControllerTop.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);

    // PID for bottom motor
    m_pidControllerBottom.setP(ShooterConstants.kShooterP[1]);
    m_pidControllerBottom.setI(ShooterConstants.kShooterI[1]);
    m_pidControllerBottom.setD(ShooterConstants.kShooterD[1]);
    m_pidControllerBottom.setFF(ShooterConstants.kShooterFF[1]);
    m_pidControllerBottom.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);

    m_shootMotorTop.burnFlash();
    m_shootMotorBottom.burnFlash();
  }

  public void shoot() {
    m_pidControllerTop.setReference(ShooterConstants.kSetPoint, CANSparkMax.ControlType.kVelocity);
    m_pidControllerBottom.setReference(ShooterConstants.kSetPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void stop() {
    m_shootMotorTop.set(0);
    m_shootMotorBottom.set(0);
  }
  
  public boolean isShooterAtSpeed() {
    double target = ShooterConstants.kSetPoint - 500;
    return m_encoderTop.getVelocity() > target && m_encoderBottom.getVelocity() > target;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shooter_Top", m_encoderTop.getVelocity());
    // SmartDashboard.putNumber("Shooter_Bottom", m_encoderBottom.getVelocity());
  }
}
