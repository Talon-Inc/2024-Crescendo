// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shootMotorTop = new CANSparkMax(ShooterConstants.kFernaggleFlabberCan1ID, ShooterConstants.kMotorType);
  private final CANSparkMax m_shootMotorBottom = new CANSparkMax(ShooterConstants.kFernaggleFlabberCan2ID, ShooterConstants.kMotorType);
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

    // initalize motor stuff
    m_shootMotorBottom.setInverted(true);

    m_pidControllerTop.setP(ShooterConstants.kShooterTopP);
    m_pidControllerTop.setI(ShooterConstants.kShooterTopI);
    m_pidControllerTop.setD(ShooterConstants.kShooterTopD);
    m_pidControllerTop.setFF(ShooterConstants.kShooterTopFF);
    m_pidControllerTop.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
  
    m_pidControllerBottom.setP(ShooterConstants.kShooterBottomP);
    m_pidControllerBottom.setI(ShooterConstants.kShooterBottomI);
    m_pidControllerBottom.setD(ShooterConstants.kShooterBottomD);
    m_pidControllerBottom.setFF(ShooterConstants.kShooterBottomFF);
    m_pidControllerBottom.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);

    m_shootMotorTop.burnFlash();
    m_shootMotorBottom.burnFlash();
  }

  public void shoot() {
    m_pidControllerTop.setReference(ShooterConstants.kSetPoint, CANSparkMax.ControlType.kVelocity);
    m_pidControllerBottom.setReference(ShooterConstants.kSetPoint, CANSparkMax.ControlType.kVelocity);

  }


  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
