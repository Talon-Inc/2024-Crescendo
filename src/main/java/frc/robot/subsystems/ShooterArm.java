// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterArm extends SubsystemBase {

  private final CANSparkMax ArmShooter1 = new CANSparkMax(ShooterConstants.ArmShooter1CanId, ShooterConstants.kMotorType);
  private final CANSparkMax Armshooter2 = new CANSparkMax(ShooterConstants.ArmShooter2CanId, ShooterConstants.kMotorType);
  /** Creates a new ShooterArm. */
  public ShooterArm() {}

  public void aimUp() {}

  public void aimDown() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
