// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.subsystems.Pneumatics;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {

  private final CANSparkMax FernaggleFlabber1 = new CANSparkMax(ShooterConstants.FernaggleFlabberCan1ID, ShooterConstants.kMotorType);
  private final CANSparkMax FernaggleFlabber2 = new CANSparkMax(ShooterConstants.FernaggleFlabberCan2ID, ShooterConstants.kMotorType);
  
  
  /** Creates a new ShooterArm. */
  public Shooter() {}

  public void SetRPM(double rpm) {
    SmartDashboard.putNumber("Desired RPM", rpm);
    /** figure out later  
    SmartDashboard.putNumber("RPM Difference", )
    */
    
  }
  public void aimUp() {}

  public void aimDown() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}