// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_FernaggleFlabber1 = new CANSparkMax(ShooterConstants.kFernaggleFlabberCan1ID, ShooterConstants.kMotorType);
  private final CANSparkMax m_FernaggleFlabber2 = new CANSparkMax(ShooterConstants.kFernaggleFlabberCan2ID, ShooterConstants.kMotorType);
  
  /** Creates a new ShooterArm. */
  public Shooter() {
    // initalize motor stuff
    m_FernaggleFlabber2.follow(m_FernaggleFlabber1);
    m_FernaggleFlabber2.setInverted(true);
  }

  public void shoot() {
    m_FernaggleFlabber1.set(0.5);
  }

  // probably garbage
  public void SetRPM(double rpm) {
    SmartDashboard.putNumber("Desired RPM", rpm);
    /** figure out later  
    SmartDashboard.putNumber("RPM Difference", )
    get velocity to smartdashboard
    SmartDashboard.putNumber("Motor1velocity", FernaggleFlabber1. );
    SmartDashboard.putNumber("Motor2velocity", FernaggleFlabber2.);
    */
    double voltage = 1 * rpm;
    m_FernaggleFlabber1.setVoltage(voltage);
    m_FernaggleFlabber2.setVoltage(-voltage);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
